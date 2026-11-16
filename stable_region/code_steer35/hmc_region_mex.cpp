// Region observation around the unchanged HMC result-model implementation.
// No alternate vehicle/tire equations are introduced in this translation unit.
#define mexFunction hmc_original_mexFunction
#include "hmc_mex.cpp"
#undef mexFunction

#include <limits>
#include <deque>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace hmc_region {
constexpr double INF=std::numeric_limits<double>::infinity();
constexpr double NANV=std::numeric_limits<double>::quiet_NaN();
constexpr double TIME_TOL=1e-8;
constexpr double DEG=180.0/PI;
constexpr int TRACE_WIDTH=57;
using TraceRow=std::array<double,TRACE_WIDTH>;

static double scalar_option(const mxArray*c,const char*k,double fallback){
 const mxArray*a=field(c,k);
 if(!a||mxIsEmpty(a))return fallback;
 if((!mxIsDouble(a)&&!mxIsLogical(a))||mxIsComplex(a)||mxGetNumberOfElements(a)!=1)
  throw std::runtime_error(std::string("Expected real scalar cfg.")+k);
 double v=mxGetScalar(a);if(!std::isfinite(v))throw std::runtime_error(std::string("Nonfinite cfg.")+k);return v;
}
static bool bool_option(const mxArray*c,const char*k,bool fallback){return scalar_option(c,k,fallback?1:0)!=0;}
static std::vector<double> vector_option(const mxArray*c,const char*k,std::vector<double> fallback){
 const mxArray*a=field(c,k);return !a||mxIsEmpty(a)?fallback:numbers(a);
}
static A4 array4(const mxArray*c,const char*k){
 auto v=numbers(required(c,k));if(v.size()!=4)throw std::runtime_error(std::string("Expected four cfg.reference.")+k);
 A4 a;for(int i=0;i<4;i++){if(!std::isfinite(v[i]))throw std::runtime_error("Nonfinite reference array");a[i]=v[i];}return a;
}
struct Handling {bool inside=false;double margin=-INF,trace=NANV,det=NANV,minC=NANV;};
struct Reference {
 double ux,m,Iz,mu_scale_y;A4 xw,yw,cd,sd,fz,fz0;
 std::array<std::array<double,12>,4>mf;
 explicit Reference(const mxArray*r){
  ux=mxGetScalar(required(r,"ux"));m=mxGetScalar(required(r,"m"));Iz=mxGetScalar(required(r,"Iz"));
  mu_scale_y=mxGetScalar(required(r,"mu_scale_y"));
  if(!std::isfinite(ux)||!std::isfinite(m)||!std::isfinite(Iz)||!std::isfinite(mu_scale_y)||ux<=0||m<=0||Iz<=0||mu_scale_y<=0)
   throw std::runtime_error("Reference ux/m/Iz/mu_scale_y must be finite and positive");
  xw=array4(r,"xw");yw=array4(r,"yw");cd=array4(r,"cdelta");sd=array4(r,"sdelta");
  fz=array4(r,"Fz_static");fz0=array4(r,"Fz0_vec");
  const mxArray*a=required(r,"MFy");auto values=numbers(a);
  if(mxGetM(a)!=4||mxGetN(a)!=12)throw std::runtime_error("Reference MFy must be 4-by-12");
  for(int i=0;i<4;i++){
   if(fz[i]<=0||fz0[i]<=0)throw std::runtime_error("Reference loads must be positive");
   for(int j=0;j<12;j++){mf[i][j]=values[i+4*j];if(!std::isfinite(mf[i][j]))throw std::runtime_error("Nonfinite MFy");}
  }
 }
 Handling evaluate(double v,double r)const{
  // Formula-for-formula port of the external cdc_handling_reference.m.
  // This common classification reference never supplies vehicle dynamics.
  double j11=0,j12=-ux,j21=0,j22=0,minC=INF,minNorm=INF;bool valid=true;
  for(int i=0;i<4;i++){
   double ub=ux-yw[i]*r,vb=v+xw[i]*r,uw=cd[i]*ub+sd[i]*vb,vw=-sd[i]*ub+cd[i]*vb;
   double alpha=std::atan2(vw,uw),df=(fz[i]-fz0[i])/fz0[i];const auto&a=mf[i];
   double D=(a[1]+a[2]*df)*fz[i]*mu_scale_y;
   double K=a[5]*fz0[i]*std::sin(a[7]*std::atan(fz[i]/(a[6]*fz0[i])));
   double B=std::abs(a[0]*D)<1e-9?0:K/(a[0]*D),E=std::min(a[3]+a[4]*df,1.0);
   double z=B*(alpha+a[8]+a[9]*df),u=z-E*(z-std::atan(z)),angle=a[0]*std::atan(u);
   double derivative=D*std::cos(angle)*a[0]/(1+u*u)*B*(1-E+E/(1+z*z));
   double denom=ub*ub+vb*vb,av=ub/denom,ar=(xw[i]*ub+yw[i]*vb)/denom;
   double b=cd[i]/m,zc=(xw[i]*cd[i]+yw[i]*sd[i])/Iz,C=-derivative;
   j11+=b*derivative*av;j12+=b*derivative*ar;j21+=zc*derivative*av;j22+=zc*derivative*ar;
   minC=std::min(minC,C);minNorm=std::min(minNorm,C/std::max(std::abs(K),1.0));
   valid=valid&&uw>.1&&std::isfinite(C);
  }
  Handling h;h.trace=j11+j22;h.det=j11*j22-j12*j21;h.minC=minC;
  valid=valid&&std::isfinite(h.trace)&&std::isfinite(h.det);
  h.inside=valid&&h.trace<0&&h.det>0&&minC>0;
  h.margin=valid?std::min(std::min(-h.trace,h.det),minNorm):-INF;return h;
 }
};
struct Config {
 Reference reference;double dt=.0005,observe_dt=.001,window=1,policy_g=9.81,mu=.6;
 double yaw_factor=.85,ay_factor=1,ltr_limit=1,roll_limit=40*PI/180,roll_limit_deg=40;
 bool early=true,window_inclusive=true,trim_inside=true,main_requires_ay=false;
 int threads=4,observe_steps=2,trace_stride=0,steps=0;
 std::array<double,7>tol{{.1,.005,.2,.2,.5,.5,.5}},target{};
 std::vector<double>horizons;std::vector<int>horizon_steps;std::vector<bool>trace_selected;
 Config(const mxArray*c,const State&trim,size_t point_count):reference(required(c,"reference")){
  if(!mxIsStruct(c)||mxGetNumberOfElements(c)!=1)throw std::runtime_error("cfg must be a scalar struct");
  dt=scalar_option(c,"dt",dt);observe_dt=scalar_option(c,"observe_dt",observe_dt);
  window=scalar_option(c,"terminal_window",window);policy_g=scalar_option(c,"policy_g",policy_g);
  mu=scalar_option(c,"mu",mu);yaw_factor=scalar_option(c,"yaw_factor",yaw_factor);
  ay_factor=scalar_option(c,"ay_factor",ay_factor);ltr_limit=scalar_option(c,"LTR_limit",ltr_limit);
  roll_limit=scalar_option(c,"roll_limit_rad",roll_limit);
  roll_limit_deg=scalar_option(c,"roll_limit_deg",roll_limit*DEG);
  early=bool_option(c,"early_reject_final",true);
  window_inclusive=bool_option(c,"window_start_inclusive",bool_option(c,"window_inclusive",true));
  main_requires_ay=bool_option(c,"main_requires_ay",false);
  trim_inside=bool_option(c,"trim_inside_reference",reference.evaluate(trim[0],trim[1]).inside);
  double nt=scalar_option(c,"threads",4),stride=scalar_option(c,"trace_stride",0);
  if(nt<1||nt>256||std::floor(nt)!=nt||stride<0||std::floor(stride)!=stride)throw std::runtime_error("Invalid threads/trace_stride");
  threads=int(nt);trace_stride=int(stride);
  if(dt<=0||observe_dt<=0||window<=0||policy_g<=0||mu<=0||yaw_factor<0||ay_factor<0||ltr_limit<=0||roll_limit<=0)
   throw std::runtime_error("Invalid positive region settings");
  observe_steps=int(std::lround(observe_dt/dt));
  if(observe_steps<1||std::abs(observe_steps*dt-observe_dt)>1e-10)throw std::runtime_error("observe_dt must lie on dt lattice");
  horizons=vector_option(c,"horizons",{5,10});if(horizons.empty())throw std::runtime_error("No horizons");
  double previous=0;
  for(double t:horizons){
   if(!std::isfinite(t)||t<=previous||t<window)throw std::runtime_error("Horizons must increase and cover terminal_window");
   long k=std::lround(t/dt);
   if(k<1||k>100000000||std::abs(k*dt-t)>1e-9||k%observe_steps!=0)throw std::runtime_error("Horizons must lie on observation lattice");
   horizon_steps.push_back(int(k));previous=t;
  }
  steps=horizon_steps.back();
  auto tolerance=vector_option(c,"terminal_tolerance",{});
  if(tolerance.empty())tolerance=vector_option(c,"terminal_tolerances",{});
  if(tolerance.empty())tolerance=vector_option(c,"tolerances",{.1,.005,.2,.2,.5,.5,.5});
  if(tolerance.size()!=7)throw std::runtime_error("Expected seven observable terminal tolerances");
  for(int i=0;i<7;i++){if(!std::isfinite(tolerance[i])||tolerance[i]<=0)throw std::runtime_error("Invalid terminal tolerance");tol[i]=tolerance[i];}
  target={trim[0],trim[1],trim[3]*DEG,trim[4]*DEG,trim[10]*DEG,trim[11]*DEG,trim[32]*3.6};
  trace_selected.assign(point_count,false);
  if(trace_stride){
   auto selected=vector_option(c,"trace_point_ids",{1});
   for(double id:selected){if(id<1||id>point_count||id!=std::floor(id))throw std::runtime_error("Invalid trace_point_ids");trace_selected[size_t(id)-1]=true;}
  }
 }
 double error(const State&x)const{
  std::array<double,7>y{{x[0],x[1],x[3]*DEG,x[4]*DEG,x[10]*DEG,x[11]*DEG,x[32]*3.6}};
  double e=0;for(int i=0;i<7;i++){if(!std::isfinite(y[i]))return INF;e=std::max(e,std::abs(y[i]-target[i])/tol[i]);}return e;
 }
 bool within_band(const State&x)const{
  std::array<double,7>y{{x[0],x[1],x[3]*DEG,x[4]*DEG,x[10]*DEG,x[11]*DEG,x[32]*3.6}};
  for(int i=0;i<7;i++)if(!std::isfinite(y[i])||std::abs(y[i]-target[i])>tol[i])return false;
  return true;
 }
};
enum Metric {FINAL,GUARDED,RECOVERABLE,MODEL_SAFE,YAW_OK,AY_OK,TERMINAL_ERROR,ENDPOINT_ERROR,
 PEAK_R,PEAK_AY,PEAK_LTR,PEAK_LTR_FRONT,PEAK_LTR_REAR,MIN_FZ,REASON,OBSERVED_UNTIL,FIRST_H,
 OUTSIDE_H,MIN_H_MARGIN,STAYED_H,FIRST_AY,FIRST_YAW,FIRST_HARD,COMPLETE,CLOSE,EARLY,EARLY_CAUSE,
 LAST_BAD,BOUNDARY_SENSITIVE,FINAL_INCLUSIVE,FINAL_EXCLUSIVE,OBSERVATION_COUNT,NMETRIC};
static const char*metric_names[NMETRIC]={"final","guarded_final","recoverable","model_safe","yaw_ok","ay_ok",
 "terminal_error","endpoint_terminal_error","peak_r","peak_ay","peak_LTR","peak_LTR_front","peak_LTR_rear",
 "min_Fz","reason","observed_until","first_handling_exit","time_outside_handling","min_handling_margin",
 "stayed_in_handling","first_ay_violation","first_yaw_violation","first_hard_violation","full_horizon_evaluated",
 "terminal_window_satisfied","early_rejected","early_rejection_cause","last_bad_time","window_boundary_sensitive",
 "final_inclusive","final_exclusive","observation_count"};
using Row=std::array<double,NMETRIC>;
struct Result {std::vector<Row>rows;State last;std::vector<TraceRow>trace;std::string error_message;};
struct PathObservation {double ltr=NANV,ltrf=NANV,ltrr=NANV;Handling h;double err=INF;uint16_t bits=0;};
static Model::Obs empty_observation(){
 Model::Obs o;o.ay=o.ax=o.throttle=NANV;
 for(A4*a:{&o.raw,&o.fz,&o.fx,&o.fy,&o.mz,&o.my,&o.mx,&o.delta,&o.gamma,&o.jounce,&o.spring,&o.damper,&o.jacking})a->fill(NANV);return o;
}
static PathObservation observe(const State&x,const State&dx,const Model::Obs&o,const Config&c){
 PathObservation z;const auto&f=o.raw;double eps=std::numeric_limits<double>::epsilon();
 z.ltr=(f[0]+f[2]-f[1]-f[3])/std::max(f[0]+f[1]+f[2]+f[3],eps);
 z.ltrf=(f[0]-f[1])/std::max(f[0]+f[1],eps);z.ltrr=(f[2]-f[3])/std::max(f[2]+f[3],eps);
 if(std::abs(x[1])>c.yaw_factor*c.mu*c.policy_g/c.reference.ux)z.bits|=1;
 if(std::abs(o.ay)>c.ay_factor*c.mu*c.policy_g)z.bits|=2;
 if(std::abs(z.ltr)>=c.ltr_limit)z.bits|=4;
 for(double v:f)if(v<=0)z.bits|=8;
 if(std::abs(z.ltrf)>=c.ltr_limit||std::abs(z.ltrr)>=c.ltr_limit)z.bits|=16;
 bool numeric=!std::isfinite(o.ay)||!std::isfinite(z.ltr)||!std::isfinite(z.ltrf)||!std::isfinite(z.ltrr);
 for(double v:f)numeric=numeric||!std::isfinite(v);
 for(double v:x)numeric=numeric||!std::isfinite(v);
 for(double v:dx)numeric=numeric||!std::isfinite(v);
 if(numeric)z.bits|=32;
 // Exact observed CarSim policy: no extra wheel-speed/slip/load-range guard.
 if(std::abs(x[3]*DEG)>=c.roll_limit_deg||x[32]<=0)z.bits|=64;
 z.h=c.reference.evaluate(x[0],x[1]);z.err=c.error(x);return z;
}
static TraceRow trace_row(size_t point,double t,const State&x,const Model::Obs&o,const PathObservation&z){
 TraceRow row{};int k=0;row[k++]=point+1;row[k++]=t;
 for(double v:x)row[k++]=v;
 row[k++]=o.ay;row[k++]=o.ax;row[k++]=o.throttle;
 for(const A4*a:{&o.raw,&o.spring,&o.damper})for(double v:*a)row[k++]=v;
 row[k++]=z.ltr;row[k++]=z.ltrf;row[k++]=z.ltrr;
 row[k++]=z.h.inside;row[k++]=z.h.margin;row[k++]=z.err;return row;
}
static Result rollout(const Model&q,State x,const State&trim,const Config&c,size_t point){
 Result out;out.rows.resize(c.horizons.size());size_t next=0;
 A4 held{};std::vector<A4>history;int pos=0,sample_steps=1,lag=0;
 if(q.ddelay>0){
  sample_steps=int(std::lround(q.dsample/c.dt));lag=int(std::lround(q.ddelay/q.dsample));
  if(sample_steps<1||lag<1||std::abs(sample_steps*c.dt-q.dsample)>1e-10||std::abs(lag*q.dsample-q.ddelay)>1e-10)
   throw std::runtime_error("CDC times must lie on integration lattice");
  State initial_derivative;Model::Obs seed;q.rhs(trim,initial_derivative,&seed);held=seed.damper;history.assign(lag,held);
 }
 uint16_t reason=0;bool alive=true,kept=true,previous_inside=true;int early_cause=0;
 double first_h=INF,first_ay=INF,first_yaw=INF,first_hard=INF,outside=0,min_margin=INF;
 double peak_r=0,peak_ay=0,peak_ltr=0,peak_ltrf=0,peak_ltrr=0,min_fz=INF;
 double last_bad=-INF,endpoint_error=INF,observed=0,last_observation=-INF;
 struct WindowSample{double time,error;bool within;};
 size_t observation_count=0;std::deque<WindowSample>errors;
 auto snapshot=[&](bool complete)->Row{
  double start=observed-c.window,inclusive_error=0,exclusive_error=0;size_t ni=0,ne=0;
  bool bands_i=true,bands_e=true;
  for(const auto&e:errors){
   if(e.time>=start-TIME_TOL){inclusive_error=std::max(inclusive_error,e.error);bands_i=bands_i&&e.within;ni++;}
   if(e.time>start+TIME_TOL){exclusive_error=std::max(exclusive_error,e.error);bands_e=bands_e&&e.within;ne++;}
  }
  if(!ni)inclusive_error=INF;if(!ne)exclusive_error=INF;
  bool covered=observed>=c.window-TIME_TOL;
  bool close_i=covered&&ni>=3&&bands_i,close_e=covered&&ne>=3&&bands_e;
  bool close=c.window_inclusive?close_i:close_e;
  bool physical=alive&&complete,yaw_ok=!(reason&1),ay_ok=!(reason&2);
  bool recovery=physical&&close,admissible=kept&&c.trim_inside&&(!c.main_requires_ay||ay_ok);
  bool fi=recovery&&admissible,fg=fi&&yaw_ok&&ay_ok;
  Row r{};r[FINAL]=fi;r[GUARDED]=fg;
  r[RECOVERABLE]=complete?double(recovery):NANV;r[MODEL_SAFE]=complete?double(physical):NANV;
  r[YAW_OK]=complete?double(yaw_ok):NANV;r[AY_OK]=complete?double(ay_ok):NANV;
  r[TERMINAL_ERROR]=c.window_inclusive?inclusive_error:exclusive_error;r[ENDPOINT_ERROR]=endpoint_error;
  r[PEAK_R]=peak_r;r[PEAK_AY]=peak_ay;r[PEAK_LTR]=peak_ltr;r[PEAK_LTR_FRONT]=peak_ltrf;r[PEAK_LTR_REAR]=peak_ltrr;
  r[MIN_FZ]=min_fz;r[REASON]=reason;r[OBSERVED_UNTIL]=observed;r[FIRST_H]=first_h;r[OUTSIDE_H]=outside;
  r[MIN_H_MARGIN]=min_margin;r[STAYED_H]=kept;r[FIRST_AY]=first_ay;r[FIRST_YAW]=first_yaw;r[FIRST_HARD]=first_hard;
  r[COMPLETE]=complete;r[CLOSE]=close;r[EARLY]=!complete;r[EARLY_CAUSE]=complete?0:early_cause;
  r[LAST_BAD]=last_bad;r[BOUNDARY_SENSITIVE]=close_i!=close_e;
  r[FINAL_INCLUSIVE]=physical&&close_i&&admissible;r[FINAL_EXCLUSIVE]=physical&&close_e&&admissible;
  r[OBSERVATION_COUNT]=observation_count;return r;
 };
 State a,b,d,e,temp;
 for(int k=0;k<=c.steps;k++){
  double t=k*c.dt;bool regular_observation=k%c.observe_steps==0;
  bool trace_here=c.trace_stride&&c.trace_selected[point]&&(k%c.trace_stride==0||k==c.steps);
  Model::Obs o=empty_observation();bool evaluation_failed=false;
  try{
   // This is the unchanged hmc_trajectory queue/RK4 ordering.
   if(lag&&k%sample_steps==0){Model::Obs command;q.rhs(x,a,&command);held=history[pos];history[pos]=command.damper;pos=(pos+1)%lag;}
   q.rhs(x,a,(regular_observation||trace_here)?&o:nullptr,lag?&held:nullptr);
   for(int j=0;j<34;j++)if(!std::isfinite(x[j])||!std::isfinite(a[j]))evaluation_failed=true;
  }catch(const std::exception&ex){evaluation_failed=true;out.error_message=ex.what();a.fill(NANV);}
  PathObservation z;
  if(regular_observation||trace_here||evaluation_failed)z=observe(x,a,o,c);
  if(evaluation_failed)z.bits|=32;
  if(regular_observation||evaluation_failed){
   if(observation_count&& !previous_inside)outside+=t-last_observation;
   observed=t;last_observation=t;observation_count++;reason|=z.bits;
   previous_inside=z.h.inside;kept=kept&&z.h.inside;
   if(!z.h.inside)first_h=std::min(first_h,t);min_margin=std::min(min_margin,z.h.margin);
   if(z.bits&1)first_yaw=std::min(first_yaw,t);if(z.bits&2)first_ay=std::min(first_ay,t);
   if(z.bits&uint16_t(4|8|16|32|64)){alive=false;first_hard=std::min(first_hard,t);}
   peak_r=std::max(peak_r,std::abs(x[1]));peak_ay=std::max(peak_ay,std::abs(o.ay));
   peak_ltr=std::max(peak_ltr,std::abs(z.ltr));peak_ltrf=std::max(peak_ltrf,std::abs(z.ltrf));peak_ltrr=std::max(peak_ltrr,std::abs(z.ltrr));
   for(double v:o.raw)min_fz=std::min(min_fz,v);
   endpoint_error=z.err;bool within=c.within_band(x);if(!within)last_bad=t;
   errors.push_back({t,z.err,within});while(!errors.empty()&&errors.front().time<t-c.window-TIME_TOL)errors.pop_front();
  }
  bool permanent=!alive||!kept||!c.trim_inside||(c.main_requires_ay&&(reason&2));
  bool stop=!alive||(c.early&&permanent);
  if(c.trace_stride&&c.trace_selected[point]&&(trace_here||stop))out.trace.push_back(trace_row(point,t,x,o,z));
  if(next<c.horizon_steps.size()&&k==c.horizon_steps[next])out.rows[next++]=snapshot(true);
  if(stop){
   if(!alive)early_cause|=4;if(!kept)early_cause|=1;if(!c.trim_inside)early_cause|=8;
   if(c.main_requires_ay&&(reason&2))early_cause|=2;
   while(next<c.horizons.size())out.rows[next++]=snapshot(false);break;
  }
  if(k==c.steps)break;
  try{
   for(int j=0;j<34;j++)temp[j]=x[j]+.5*c.dt*a[j];q.rhs(temp,b,nullptr,lag?&held:nullptr);
   for(int j=0;j<34;j++)temp[j]=x[j]+.5*c.dt*b[j];q.rhs(temp,d,nullptr,lag?&held:nullptr);
   for(int j=0;j<34;j++)temp[j]=x[j]+c.dt*d[j];q.rhs(temp,e,nullptr,lag?&held:nullptr);
   for(int j=0;j<34;j++)x[j]+=c.dt/6*(a[j]+2*b[j]+2*d[j]+e[j]);
  }catch(const std::exception&ex){
   out.error_message=ex.what();reason|=32;alive=false;early_cause|=4;observed=t;first_hard=std::min(first_hard,t);
   while(next<c.horizons.size())out.rows[next++]=snapshot(false);break;
  }
 }
 out.last=x;return out;
}

static void run(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 if(nrhs!=5||nlhs!=1)throw std::runtime_error("out=hmc_region_mex('region_rollout',h,X0,trim,cfg)");
 const Model&q=*handle(prhs[1]);const mxArray*input=prhs[2];
 if(mxGetM(input)!=34)throw std::runtime_error("X0 must be 34-by-N");
 auto values=numbers(input),tv=numbers(prhs[3]);size_t n=mxGetN(input);
 if(n<1||tv.size()!=34)throw std::runtime_error("Expected nonempty initial states and 34-state trim");
 for(double v:values)if(!std::isfinite(v))throw std::runtime_error("X0 must be finite");
 State trim;for(int j=0;j<34;j++){if(!std::isfinite(tv[j]))throw std::runtime_error("Trim must be finite");trim[j]=tv[j];}
 Config c(prhs[4],trim,n);std::vector<Result>results(n);std::vector<std::string>failures(n);
 #ifdef _OPENMP
 #pragma omp parallel for schedule(dynamic,1) num_threads(c.threads)
 #endif
 for(long long i=0;i<static_cast<long long>(n);i++){
  try{State x;std::copy(values.begin()+i*34,values.begin()+(i+1)*34,x.begin());results[i]=rollout(q,x,trim,c,size_t(i));}
  catch(const std::exception&ex){failures[i]=ex.what();}
 }
 for(size_t i=0;i<n;i++)if(!failures[i].empty())throw std::runtime_error("Region point "+std::to_string(i+1)+": "+failures[i]);
 std::vector<const char*>keys(metric_names,metric_names+NMETRIC);
 for(const char*k:{"final_state","trace","trace_columns","error_message","schema","dt_s","observe_dt_s","horizons_s","terminal_tolerance","terminal_target","window_start_inclusive","main_requires_ay","reference_definition"})keys.push_back(k);
 plhs[0]=mxCreateStructMatrix(1,1,int(keys.size()),keys.data());
 for(int j=0;j<NMETRIC;j++){
  mxArray*a=mxCreateDoubleMatrix(n,c.horizons.size(),mxREAL);double*p=mxGetPr(a);
  for(size_t i=0;i<n;i++)for(size_t k=0;k<c.horizons.size();k++)p[i+n*k]=results[i].rows[k][j];
  mxSetField(plhs[0],0,metric_names[j],a);
 }
 mxArray*states=mxCreateDoubleMatrix(34,n,mxREAL);for(size_t i=0;i<n;i++)std::copy(results[i].last.begin(),results[i].last.end(),mxGetPr(states)+34*i);mxSetField(plhs[0],0,"final_state",states);
 size_t nt=0;for(const auto&r:results)nt+=r.trace.size();mxArray*trace=mxCreateDoubleMatrix(nt,TRACE_WIDTH,mxREAL);size_t row_index=0;
 for(const auto&r:results)for(const auto&row:r.trace){for(int j=0;j<TRACE_WIDTH;j++)mxGetPr(trace)[row_index+nt*j]=row[j];row_index++;}
 mxSetField(plhs[0],0,"trace",trace);
 std::vector<std::string>trace_names={"point_index","time_s"};for(int j=1;j<=34;j++)trace_names.push_back("state_"+std::to_string(j));
 for(const char*k:{"ay_mps2","ax_mps2","throttle","Fz_FL","Fz_FR","Fz_RL","Fz_RR","spring_FL","spring_FR","spring_RL","spring_RR","damper_FL","damper_FR","damper_RL","damper_RR","LTR","LTR_front","LTR_rear","inside_H","H_margin","endpoint_terminal_error"})trace_names.push_back(k);
 mxArray*trace_columns=mxCreateCellMatrix(1,TRACE_WIDTH);for(int j=0;j<TRACE_WIDTH;j++)mxSetCell(trace_columns,j,mxCreateString(trace_names[j].c_str()));mxSetField(plhs[0],0,"trace_columns",trace_columns);
 mxArray*messages=mxCreateCellMatrix(n,1);for(size_t i=0;i<n;i++)mxSetCell(messages,i,mxCreateString(results[i].error_message.c_str()));mxSetField(plhs[0],0,"error_message",messages);
 mxSetField(plhs[0],0,"schema",mxCreateString("HMC_RESULT_MODEL_CARSIM_OUTPUT_REGION_V1"));
 mxSetField(plhs[0],0,"dt_s",scalar(c.dt));mxSetField(plhs[0],0,"observe_dt_s",scalar(c.observe_dt));
 mxSetField(plhs[0],0,"horizons_s",row(c.horizons.data(),c.horizons.size()));
 mxSetField(plhs[0],0,"terminal_tolerance",row(c.tol.data(),7));mxSetField(plhs[0],0,"terminal_target",row(c.target.data(),7));
 mxSetField(plhs[0],0,"window_start_inclusive",mxCreateLogicalScalar(c.window_inclusive));mxSetField(plhs[0],0,"main_requires_ay",mxCreateLogicalScalar(c.main_requires_ay));
 mxSetField(plhs[0],0,"reference_definition",mxCreateString("External common static MF H; trace<0, determinant>0, all -dFy/dalpha>0. Vehicle RHS always unchanged HMC result model."));
}
} // namespace hmc_region

extern "C" void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 try{
  if(nrhs>0&&mxIsChar(prhs[0])&&mxstr(prhs[0])=="region_rollout")hmc_region::run(nlhs,plhs,nrhs,prhs);
  else hmc_original_mexFunction(nlhs,plhs,nrhs,prhs);
 }catch(const std::exception&ex){mexErrMsgIdAndTxt("HMC:Region", "%s",ex.what());}
}
