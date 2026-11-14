// Optional MATLAB accelerator for the accompanying independent vehicle model.
// Build in MATLAB: mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17','hmc_mex.cpp')
// Windows: mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17','hmc_mex.cpp')
// This file includes hmc_native.cpp and requires tire_fields.inc beside it.
#include "mex.h"
#include "hmc_native.cpp"
#include <set>
#include <cstdint>

static std::set<Model*> live_models;
static bool cleanup_registered=false;
static void clean_all(){for(Model*q:live_models)delete q;live_models.clear();}
static std::string mxstr(const mxArray*a){
 if(!a||!mxIsChar(a))throw std::runtime_error("Expected a character vector; use char() for strings");
 char*p=mxArrayToString(a);if(!p)throw std::runtime_error("Cannot decode text");std::string s(p);mxFree(p);return s;
}
static const mxArray*field(const mxArray*a,const char*k){return a&&mxIsStruct(a)?mxGetField(a,0,k):nullptr;}
static const mxArray*required(const mxArray*a,const char*k){const mxArray*v=field(a,k);if(!v)throw std::runtime_error(std::string("Missing field: ")+k);return v;}
static std::vector<double>numbers(const mxArray*a){
 if(!a||mxIsComplex(a)||(!mxIsDouble(a)&&!mxIsLogical(a)))throw std::runtime_error("Expected real double or logical data");
 size_t n=mxGetNumberOfElements(a);std::vector<double>v(n);
 if(mxIsDouble(a)){const double*p=mxGetPr(a);std::copy(p,p+n,v.begin());}
 else{const mxLogical*p=mxGetLogicals(a);for(size_t i=0;i<n;i++)v[i]=p[i]?1.:0.;}return v;
}
static double option_number(const mxArray*a,const char*k,double def){const mxArray*v=field(a,k);return v&&!mxIsEmpty(v)?mxGetScalar(v):def;}
static void set_options(Model&q,const mxArray*c){
 if(!c||mxIsEmpty(c))return;if(!mxIsStruct(c))throw std::runtime_error("Options must be a struct");
 for(int j=0;j<mxGetNumberOfFields(c);j++){const char*key=mxGetFieldNameByNumber(c,j);const mxArray*v=mxGetFieldByNumber(c,0,j);if(!v||mxIsEmpty(v))continue;
  if((mxIsNumeric(v)||mxIsLogical(v))&&mxGetNumberOfElements(v)==1)q.options[key]=mxGetScalar(v);
  else if(std::strcmp(key,"vertical_model")==0)q.options[key]=mxstr(v)=="mf62"?1:0;
  else if(std::strcmp(key,"angular_model")==0){auto s=mxstr(v);q.options[key]=s=="legacy"?0:(s=="euler_gyro"?2:1);}
 }
}
static Model*build(const mxArray*D,const mxArray*c){
 Model*q=new Model();try{
  const mxArray*V=required(D,"vehicle");
  for(int j=0;j<mxGetNumberOfFields(V);j++){const char*k=mxGetFieldNameByNumber(V,j);const mxArray*v=mxGetFieldByNumber(V,0,j);if(v&&!mxIsEmpty(v)&&(mxIsDouble(v)||mxIsLogical(v)))q->v[k]=numbers(v);}
  const mxArray*compliance=required(V,"compliance");for(int j=0;j<mxGetNumberOfFields(compliance);j++){const char*k=mxGetFieldNameByNumber(compliance,j);q->v[k]=numbers(required(mxGetFieldByNumber(compliance,0,j),"SI"));}
  auto sm=field(c,"spring_modes");q->v["spring_modes"]=sm?numbers(sm):std::vector<double>(4,2.);
  std::vector<double>wear(4,0.);if(field(c,"wear"))wear=numbers(field(c,"wear"));if(wear.size()!=4)throw std::runtime_error("wear needs FL FR RL RR values");
  const mxArray*alltires=required(D,"tires");
  for(int i=0;i<4;i++){std::string name;if(wear[i]==0){std::string alias=i<2?"new_front":"new_rear";name=field(alltires,alias.c_str())?alias:"new";}else name=std::string(i<2?"front":"rear")+std::to_string(int(wear[i]));const mxArray*t=required(alltires,name.c_str());for(int j=0;j<mxGetNumberOfFields(t);j++){const mxArray*v=mxGetFieldByNumber(t,0,j);if(v&&mxIsNumeric(v)&&mxGetNumberOfElements(v)==1)q->tires[i].set(mxGetFieldNameByNumber(t,j),mxGetScalar(v));}q->tires[i].SIDE=1;}
  const char*corners[4]={"FL","FR","RL","RR"};const mxArray*tables=required(V,"tables");
  for(int g=0;g<NGROUPS;g++){const mxArray*group=required(tables,groupnames[g]);for(int i=0;i<(g==STEERING?2:4);i++){const mxArray*t=required(group,corners[i]);const mxArray*xa=nullptr,*ya=nullptr;for(int j=0;j<mxGetNumberOfFields(t);j++){const char*k=mxGetFieldNameByNumber(t,j);if(std::strncmp(k,"x_",2)==0&&!xa)xa=mxGetFieldByNumber(t,0,j);if(std::strncmp(k,"y_",2)==0&&!ya)ya=mxGetFieldByNumber(t,0,j);}auto x=numbers(xa),y=numbers(ya);if(x.size()!=y.size())throw std::runtime_error("Table x/y size mismatch");bool spline=field(t,"interpolation")&&mxstr(field(t,"interpolation"))=="SPLINE";q->tabs[g][i].setup(x.data(),y.data(),int(x.size()),spline);}}
  const mxArray*d=required(D,"damper");auto current=numbers(required(d,"current_mA")),vel=numbers(required(d,"v_mm_s"));std::vector<double>cmd(4,950);if(field(c,"I_mA")){cmd=numbers(field(c,"I_mA"));if(cmd.size()==1)cmd=std::vector<double>(4,cmd[0]);}if(cmd.size()!=4)throw std::runtime_error("I_mA needs one or four values");
  for(int i=0;i<4;i++){const mxArray*map=required(d,i<2?"front_N":"rear_N");auto mat=numbers(map);if(mxGetM(map)!=current.size()||mxGetN(map)!=vel.size())throw std::runtime_error("Damper map must be current-by-velocity");std::vector<double>values(vel.size());for(size_t j=0;j<vel.size();j++){Table t;t.setup(current.data(),mat.data()+current.size()*j,int(current.size()),false);values[j]=t.value(cmd[i],true);}q->damp[i].setup(vel.data(),values.data(),int(vel.size()),false);}
  set_options(*q,c);q->prepare();return q;
 }catch(...){delete q;throw;}
}
static Model*handle(const mxArray*a){
 if(!a||!mxIsUint64(a)||mxGetNumberOfElements(a)!=1)throw std::runtime_error("Expected a live uint64 native handle");auto*q=reinterpret_cast<Model*>(*static_cast<uint64_T*>(mxGetData(a)));if(live_models.find(q)==live_models.end())throw std::runtime_error("Native handle is not live; create a new one");return q;
}
static mxArray*scalar(double x){return mxCreateDoubleScalar(x);}
static mxArray*row(const double*v,size_t n){mxArray*a=mxCreateDoubleMatrix(1,n,mxREAL);std::copy(v,v+n,mxGetPr(a));return a;}
static mxArray*col(const double*v,size_t n){mxArray*a=mxCreateDoubleMatrix(n,1,mxREAL);std::copy(v,v+n,mxGetPr(a));return a;}
static mxArray*observation(const Model::Obs&o){
 const char*keys[]={"ay","ax","throttle","Fz_raw","Fz","Fx","Fy","Mz","My","Mx","delta","gamma","jounce","spring_force","damper_force","jacking_force"};mxArray*a=mxCreateStructMatrix(1,1,16,keys);
 mxSetField(a,0,"ay",scalar(o.ay));mxSetField(a,0,"ax",scalar(o.ax));mxSetField(a,0,"throttle",scalar(o.throttle));int k=3;for(const auto&v:{o.raw,o.fz,o.fx,o.fy,o.mz,o.my,o.mx,o.delta,o.gamma,o.jounce,o.spring,o.damper,o.jacking})mxSetField(a,0,keys[k++],col(v.data(),4));return a;
}
void mexFunction(int nlhs,mxArray*plhs[],int nrhs,const mxArray*prhs[]){
 try{
  if(nrhs<1)throw std::runtime_error("Supply an action");std::string action=mxstr(prhs[0]);
  if(action=="new"){
   if(nrhs<2||nlhs!=1)throw std::runtime_error("h = hmc_mex('new',D,options)");Model*q=build(prhs[1],nrhs>2?prhs[2]:nullptr);live_models.insert(q);mexLock();if(!cleanup_registered){mexAtExit(clean_all);cleanup_registered=true;}plhs[0]=mxCreateNumericMatrix(1,1,mxUINT64_CLASS,mxREAL);*static_cast<uint64_T*>(mxGetData(plhs[0]))=reinterpret_cast<uint64_T>(q);return;
  }
  if(nrhs<2)throw std::runtime_error("Supply model handle");Model*q=handle(prhs[1]);
  if(action=="delete"){live_models.erase(q);delete q;mexUnlock();return;}
  if(action=="set_options"){
   if(nrhs<3)throw std::runtime_error("Supply options struct");set_options(*q,prhs[2]);q->prepare();return;
  }
  if(action=="set_vehicle"){
   if(nrhs<4)throw std::runtime_error("Supply vehicle key and numeric value");q->v[mxstr(prhs[2])]=numbers(prhs[3]);q->prepare();return;
  }
  if(action=="trimstate"){
   if(nrhs<3||nlhs!=1||mxGetM(prhs[2])!=18)throw std::runtime_error("X = hmc_mex('trimstate',h,Y); Y must be18-by-N");auto y=numbers(prhs[2]);size_t n=mxGetN(prhs[2]);plhs[0]=mxCreateDoubleMatrix(34,n,mxREAL);double*out=mxGetPr(plhs[0]);for(size_t j=0;j<n;j++){State x;q->trimstate(y.data()+j*18,x);std::copy(x.begin(),x.end(),out+j*34);}return;
  }
  if(action=="tire_force"){
   if(nrhs!=8||nlhs!=1)throw std::runtime_error("F = hmc_mex('tire_force',h,corner,kappa,alpha,Fz,gamma,Vx); corner=1..4");int i=int(mxGetScalar(prhs[2]))-1;double out[7];if(hmc_tire_force(q,i,mxGetScalar(prhs[3]),mxGetScalar(prhs[4]),mxGetScalar(prhs[5]),mxGetScalar(prhs[6]),mxGetScalar(prhs[7]),out))throw std::runtime_error(hmc_error());plhs[0]=col(out,7);return;
  }
  if(action=="rhs"){
   if(nrhs<3||nlhs<1||nlhs>2||mxGetM(prhs[2])!=34)throw std::runtime_error("[dX,O] = hmc_mex('rhs',h,X[,damper_force]); X must be34-by-N");auto xx=numbers(prhs[2]);size_t n=mxGetN(prhs[2]);if(nlhs>1&&n!=1)throw std::runtime_error("Observation struct is available for one state column");std::vector<double>fd;if(nrhs>3&&!mxIsEmpty(prhs[3])){fd=numbers(prhs[3]);if(fd.size()!=4&&fd.size()!=4*n)throw std::runtime_error("Damper override must be4-by-1 or4-by-N");}
   plhs[0]=mxCreateDoubleMatrix(34,n,mxREAL);double*out=mxGetPr(plhs[0]);for(size_t j=0;j<n;j++){State x,dx;std::copy(xx.data()+j*34,xx.data()+(j+1)*34,x.begin());A4 over;if(!fd.empty())std::copy(fd.data()+(fd.size()==4?0:4*j),fd.data()+(fd.size()==4?4:4*j+4),over.begin());Model::Obs o;q->rhs(x,dx,nlhs>1?&o:nullptr,fd.empty()?nullptr:&over);std::copy(dx.begin(),dx.end(),out+j*34);if(nlhs>1)plhs[1]=observation(o);}return;
  }
  if(action=="trajectory"){
   if(nrhs<6||nlhs!=1)throw std::runtime_error("tr=hmc_mex('trajectory',h,X0,Xtrim,dt,duration)");auto x=numbers(prhs[2]),trim=numbers(prhs[3]);double dt=mxGetScalar(prhs[4]),duration=mxGetScalar(prhs[5]);if(x.size()!=34||(!trim.empty()&&trim.size()!=34)||!std::isfinite(dt)||dt<=0||!std::isfinite(duration)||duration<=0)throw std::runtime_error("Invalid trajectory state or time settings");int steps=std::lround(duration/dt);if(std::abs(steps*dt-duration)>1e-9)throw std::runtime_error("Duration must lie on integration lattice");size_t n=steps+1;std::vector<double>states(n*34),obs(n*15);if(hmc_trajectory(q,x.data(),trim.empty()?nullptr:trim.data(),dt,steps,states.data(),obs.data()))throw std::runtime_error(hmc_error());
   const char*keys[]={"time_s","state","ay_mps2","ax_mps2","throttle","Fz_raw_N","spring_force_N","damper_force_N"};plhs[0]=mxCreateStructMatrix(1,1,8,keys);mxArray*time=mxCreateDoubleMatrix(1,n,mxREAL),*state=mxCreateDoubleMatrix(34,n,mxREAL);for(size_t j=0;j<n;j++)mxGetPr(time)[j]=j*dt;std::copy(states.begin(),states.end(),mxGetPr(state));mxSetField(plhs[0],0,"time_s",time);mxSetField(plhs[0],0,"state",state);
   for(int k=0;k<3;k++){mxArray*a=mxCreateDoubleMatrix(1,n,mxREAL);for(size_t j=0;j<n;j++)mxGetPr(a)[j]=obs[j*15+k];mxSetField(plhs[0],0,keys[2+k],a);}
   for(int k=0;k<3;k++){mxArray*a=mxCreateDoubleMatrix(4,n,mxREAL);for(size_t j=0;j<n;j++)for(int i=0;i<4;i++)mxGetPr(a)[j*4+i]=obs[j*15+3+4*k+i];mxSetField(plhs[0],0,keys[5+k],a);}return;
  }
  throw std::runtime_error("Unknown action: "+action);
 }catch(const std::exception&e){mexErrMsgIdAndTxt("HMC:Native", "%s",e.what());}
}
