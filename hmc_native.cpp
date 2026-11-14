#include <cmath>
#include <algorithm>
#include <array>
#include <vector>
#include <map>
#include <string>
#include <stdexcept>
#include <cstring>
#include <cstdio>

// Scalar, equation-for-equation translation of HMC_TRANSIENT_RESPONSE_V3.
// This is an analysis engine, with no CarSim-reference data in its dynamics.
static constexpr double PI=3.1415926535897932384626433832795;
using A4=std::array<double,4>;
using State=std::array<double,34>;
static inline double sq(double a){return a*a;}
static inline double sign(double a){return (a>0)-(a<0);}
static inline double den(double a){return std::abs(a)<1e-12?(a<0?-1e-12:1e-12):a;}
static inline double wave(double x,double e){return x-e*(x-std::atan(x));}
static inline double clamp(double v,double lo,double hi){return std::min(std::max(v,lo),hi);}

struct Tire {
 bool mf62_correct_trail=false;
#define X(k,d) double k=d;
#include "tire_fields.inc"
#undef X
 void set(const std::string& key,double val){
#define X(k,d) if(key==#k){k=val;return;}
#include "tire_fields.inc"
#undef X
 }
};
struct TF {double Fx=0,Fy=0,Mz=0,My=0,Mx=0,sx=0,sy=0;};
static std::array<double,2> relaxation(double fz,double g,double vx,double pressure,const Tire&t,double mu){
 if(!std::isfinite(fz)||!std::isfinite(g)||!std::isfinite(vx)||!std::isfinite(pressure)||!std::isfinite(mu)||pressure<=0||mu<=0||fz<=0)return {0,0};
 const double f0=t.FNOMIN*t.LFZO,gst=t.SIDE*g,df=(fz-f0)/f0,dp=(pressure-t.NOMPRES)/t.NOMPRES;
 double Kx=fz*(t.PKX1+t.PKX2*df)*std::exp(t.PKX3*df)*(1+t.PPX1*dp+t.PPX2*dp*dp)*t.LKX;
 double Ky=t.PKY1*f0*(1+t.PPY1*dp)*(1-t.PKY3*std::abs(gst))*std::sin(t.PKY4*std::atan(fz/den((t.PKY2+t.PKY5*gst*gst)*f0*(1+t.PPY2*dp))))*t.LKY;
 double Cfx=t.LONGITUDINAL_STIFFNESS*(1+t.PCFX1*df+t.PCFX2*df*df)*(1+t.PCFX3*dp),Cfy=t.LATERAL_STIFFNESS*(1+t.PCFY1*df+t.PCFY2*df*df)*(1+t.PCFY3*dp);
 return {std::abs(Kx)/den(Cfx),std::abs(Ky)/den(Cfy)};
}
static TF tire_force(double k,double a,double fz,double g,double vx,double pressure,const Tire&t,double mu){
 TF o;bool valid=std::isfinite(k)&&std::isfinite(a)&&std::isfinite(fz)&&std::isfinite(g)&&std::isfinite(vx)&&std::isfinite(pressure)&&std::isfinite(mu)&&pressure>0&&mu>=0;
 if(!valid||fz<=0||mu==0)return o;
 const double r0=t.UNLOADED_RADIUS,f0=t.FNOMIN*t.LFZO,pn=t.NOMPRES,side=t.SIDE;
 const double direction=vx<0?-1:1,ast=side*a*direction,gst=side*g,df=(fz-f0)/f0,dp=(pressure-pn)/pn;
 const double mux=t.LMUX*mu,muy=t.LMUY*mu,cx=t.PCX1*t.LCX;
 double mu_x=(t.PDX1+t.PDX2*df)*(1+t.PPX3*dp+t.PPX4*dp*dp)*(1-t.PDX3*gst*gst)*mux;
 double Dx=mu_x*fz,Kx=fz*(t.PKX1+t.PKX2*df)*std::exp(t.PKX3*df)*(1+t.PPX1*dp+t.PPX2*dp*dp)*t.LKX;
 double shx=(t.PHX1+t.PHX2*df)*t.LHX,svx=fz*(t.PVX1+t.PVX2*df)*t.LVX*mux,kxp=k+shx;
 double ex=std::min((t.PEX1+t.PEX2*df+t.PEX3*df*df)*(1-t.PEX4*sign(kxp))*t.LEX,1.0);
 double bx=Kx/den(cx*Dx),fx0=Dx*std::sin(cx*std::atan(wave(bx*kxp,ex)))+svx;
 double bxa=(t.RBX1+t.RBX3*gst*gst)*std::cos(std::atan(t.RBX2*k))*t.LXAL,exa=std::min(t.REX1+t.REX2*df,1.0);
 double gxa=std::cos(t.RCX1*std::atan(wave(bxa*(ast+t.RHX1),exa)))/den(std::cos(t.RCX1*std::atan(wave(bxa*t.RHX1,exa))));
 o.Fx=fx0*gxa;
 double cy=t.PCY1*t.LCY,mu_y=(t.PDY1+t.PDY2*df)*(1+t.PPY3*dp+t.PPY4*dp*dp)*(1-t.PDY3*gst*gst)*muy,Dy=mu_y*fz;
 double Ky=t.PKY1*f0*(1+t.PPY1*dp)*(1-t.PKY3*std::abs(gst))*std::sin(t.PKY4*std::atan(fz/den((t.PKY2+t.PKY5*gst*gst)*f0*(1+t.PPY2*dp))))*t.LKY;
 double kyg=fz*(t.PKY6+t.PKY7*df)*(1+t.PPY5*dp)*t.LKYC,svyg=fz*(t.PVY3+t.PVY4*df)*gst*t.LKYC*muy;
 double shy=(t.PHY1+t.PHY2*df)*t.LHY+(kyg*gst-svyg)/den(Ky),svy=fz*(t.PVY1+t.PVY2*df)*t.LVY*muy+svyg;
 double ay=ast+shy,ey=std::min((t.PEY1+t.PEY2*df)*(1+t.PEY5*gst*gst-(t.PEY3+t.PEY4*gst)*sign(ay))*t.LEY,1.0);
 double by=Ky/den(cy*Dy),fy0=Dy*std::sin(cy*std::atan(wave(by*ay,ey)))+svy;
 double byk=(t.RBY1+t.RBY4*gst*gst)*std::cos(std::atan(t.RBY2*(ast-t.RBY3)))*t.LYKA;
 double eyk=std::min(t.REY1+t.REY2*df,1.0),shyk=t.RHY1+t.RHY2*df;
 double gyk=std::cos(t.RCY1*std::atan(wave(byk*(k+shyk),eyk)))/den(std::cos(t.RCY1*std::atan(wave(byk*shyk,eyk))));
 double dvyk=mu_y*fz*(t.RVY1+t.RVY2*df+t.RVY3*gst)*std::cos(std::atan(t.RVY4*ast));
 double svyk=dvyk*std::sin(t.RVY5*std::atan(t.RVY6*k))*t.LVYKA,fylocal=fy0*gyk+svyk;
 double sht=t.QHZ1+t.QHZ2*df+(t.QHZ3+t.QHZ4*df)*gst,at=ast+sht,ar=ast+shy+svy/den(Ky);
 double bt=(t.QBZ1+t.QBZ2*df+t.QBZ3*df*df)*(1+t.QBZ4*gst+t.QBZ5*std::abs(gst))*t.LKY/den(muy),ct=t.QCZ1;
 double dt=(t.QDZ1+t.QDZ2*df)*(1-t.PPZ1*dp)*fz*r0/f0*t.LTR*(1+t.QDZ3*(t.mf62_correct_trail?gst:std::abs(gst))+t.QDZ4*gst*gst)*direction;
 double et=std::min((t.QEZ1+t.QEZ2*df+t.QEZ3*df*df)*(1+(t.QEZ4+t.QEZ5*gst)*(2/PI)*std::atan(bt*ct*at)),1.0);
 double kr=Kx/den(Ky),ateq=std::atan(std::hypot(std::tan(at),kr*k))*sign(at),areq=std::atan(std::hypot(std::tan(ar),kr*k))*sign(ar);
 double trail=dt*std::cos(ct*std::atan(wave(bt*ateq,et)))*std::cos(a),br=t.QBZ9*t.LKY/den(muy)+t.QBZ10*by*cy;
 double dr=fz*r0*((t.QDZ6+t.QDZ7*df)*t.LRES+(t.QDZ8+t.QDZ9*df)*(1+t.PPZ2*dp)*gst*t.LKZC+(t.QDZ10+t.QDZ11*df)*gst*std::abs(gst))*muy*direction*std::cos(a);
 double mzr=dr*std::cos(std::atan(br*areq)),lever=r0*(t.SSZ1+t.SSZ2*fylocal/f0+(t.SSZ3+t.SSZ4*df)*gst)*t.LS;
 double trail_force=fylocal-svyk;
 if(t.mf62_correct_trail){
  // MF6.2 uses pure lateral force and combined-slip reduction evaluated
  // at zero camber/turn-slip in the pneumatic-trail product. Actual camber
  // remains in trail, residual aligning torque and longitudinal-force lever.
  double Dy0=(t.PDY1+t.PDY2*df)*(1+t.PPY3*dp+t.PPY4*dp*dp)*muy*fz;
  double Ky0=t.PKY1*f0*(1+t.PPY1*dp)*std::sin(t.PKY4*std::atan(fz/den(t.PKY2*f0*(1+t.PPY2*dp))))*t.LKY;
  double shy0=(t.PHY1+t.PHY2*df)*t.LHY,svy0=fz*(t.PVY1+t.PVY2*df)*t.LVY*muy,ay0=ast+shy0;
  double ey0=std::min((t.PEY1+t.PEY2*df)*(1-t.PEY3*sign(ay0))*t.LEY,1.0),by0=Ky0/den(cy*Dy0);
  double fy00=Dy0*std::sin(cy*std::atan(wave(by0*ay0,ey0)))+svy0;
  double byk0=t.RBY1*std::cos(std::atan(t.RBY2*(ast-t.RBY3)))*t.LYKA;
  double gyk0=std::cos(t.RCY1*std::atan(wave(byk0*(k+shyk),eyk)))/den(std::cos(t.RCY1*std::atan(wave(byk0*shyk,eyk))));
  trail_force=fy00*gyk0;
 }
 o.Mz=side*(-trail*trail_force+mzr+lever*o.Fx);o.Fy=side*fylocal;
 double gx=side*g;
 o.Mx=side*r0*fz*t.LMX*(t.QSX1*t.LVMX-t.QSX2*gx*(1+t.PPMX1*dp)+t.QSX3*fylocal/f0+t.QSX4*std::cos(t.QSX5*std::atan(sq(t.QSX6*fz/f0)))*std::sin(t.QSX7*gx+t.QSX8*std::atan(t.QSX9*fylocal/f0))+t.QSX10*std::atan(t.QSX11*fz/f0)*gx)+side*r0*fylocal*t.LMX*(t.QSX13+t.QSX14*std::abs(gx));
 double vnorm=std::abs(vx/t.LONGVL);
 o.My=-r0*f0*(t.QSY1+t.QSY2*o.Fx/f0+t.QSY3*vnorm+t.QSY4*std::pow(vnorm,4)+(t.QSY5+t.QSY6*fz/f0)*gst*gst)*std::pow(fz/f0,t.QSY7)*std::pow(pressure/pn,t.QSY8)*t.LMY*sign(vx);
 double Cfx=t.LONGITUDINAL_STIFFNESS*(1+t.PCFX1*df+t.PCFX2*df*df)*(1+t.PCFX3*dp),Cfy=t.LATERAL_STIFFNESS*(1+t.PCFY1*df+t.PCFY2*df*df)*(1+t.PCFY3*dp);
 o.sx=std::abs(Kx)/den(Cfx);o.sy=std::abs(Ky)/den(Cfy);return o;
}

struct Table {
 std::vector<double>x,y,b,c,d;bool spline=false;
 void setup(const double*xx,const double*yy,int n,bool sp){
  if(n<2)throw std::runtime_error("Table needs two points");x.assign(xx,xx+n);y.assign(yy,yy+n);spline=sp;b.clear();c.clear();d.clear();
  if(!sp)return;std::vector<double>h(n-1),v(n-1),M(n,0);
  for(int i=0;i<n-1;i++){h[i]=x[i+1]-x[i];v[i]=(y[i+1]-y[i])/h[i];}
  if(n>2){std::vector<double>aa(n-2),bb(n-2),cc(n-2),dd(n-2);
   for(int i=0;i<n-2;i++){aa[i]=h[i];bb[i]=2*(h[i]+h[i+1]);cc[i]=h[i+1];dd[i]=6*(v[i+1]-v[i]);}
   for(int j=1;j<n-2;j++){double z=aa[j]/bb[j-1];bb[j]-=z*cc[j-1];dd[j]-=z*dd[j-1];}
   M[n-2]=dd[n-3]/bb[n-3];for(int j=n-4;j>=0;j--)M[j+1]=(dd[j]-cc[j]*M[j+2])/bb[j];
  }
  for(int i=0;i<n-1;i++){b.push_back(v[i]-h[i]*(2*M[i]+M[i+1])/6);c.push_back(M[i]/2);d.push_back((M[i+1]-M[i])/(6*h[i]));}
 }
 int index(double q)const{return std::max(0,std::min(int(x.size())-2,int(std::upper_bound(x.begin()+1,x.end()-1,q)-x.begin())-1));}
 double value(double q,bool cl=false)const{
  if(x.size()<2)throw std::runtime_error("Uninitialized table");if(cl)q=clamp(q,x.front(),x.back());int j=index(q);double z=q-x[j];
  if(!spline)return y[j]+z*(y[j+1]-y[j])/(x[j+1]-x[j]);
  if(q<x.front())return y.front()+b.front()*(q-x.front());
  if(q>x.back())return y.back()+slope(x.back())*(q-x.back());
  return y[j]+z*(b[j]+z*(c[j]+z*d[j]));
 }
 double slope(double q)const{
  int j=index(q);if(!spline)return(y[j+1]-y[j])/(x[j+1]-x[j]);
  if(q<x.front())return b.front();if(q>x.back())q=x.back();double z=q-x[j];return b[j]+2*c[j]*z+3*d[j]*z*z;
 }
};
enum Group{SPRING_COMP,SPRING_EXT,JOUNCE_STOP,REBOUND_STOP,CAMBER,TOE,DIVE,SUSP_X,SUSP_LAT,STEERING,NGROUPS};
static const char* groupnames[NGROUPS]={"spring_comp","spring_ext","jounce_stop","rebound_stop","camber","toe","dive","susp_x","susp_lat","steering"};

struct Model{
 std::map<std::string,std::vector<double>>v;std::map<std::string,double>options;
 std::array<Tire,4>tires;std::array<std::array<Table,4>,NGROUPS>tabs;std::array<Table,4>damp;
 double ms,m,g,Ixx,Iyy,Iz,L,lf,lr,tf,tr,h,Kaf,Kar,Caf,Car,KP,KI,KP3,power,rear_drive,rho,area,Cx,steering_ratio,gear_ratio,ux_target,mu,steer,Iz_total;
 A4 mu_u,Iw,wheelheight,camberstatic,toestatic,caster,kpi,side{{1,-1,1,-1}},xw,yw,Fs0,Fz0,kt,ct,pressure,Re0,comp0,j0,springrate;
 A4 CT_FX,CS_FY,CS_MZ,CC_FX,CI_FY,CI_MZ;
 bool deflect=true,ldeflect=false,source_drive=true,rollgeom=true,spatial=true,compliance_body=true,steering_camber_on=true,full_orientation=false,heading=true,coupled=true,coupled_pitch=false,support_projection=false,wheel_spin_gyro=false,corrected_cc_fx_sign=false,vertical=true,jacking=true,rolling=true,compliance=true;
 int angular=2; // 0 legacy, 1 Euler kinematics, 2 Euler plus gyroscopic terms
 double ddelay=.002,dsample=.001;
 double extra_roll_damping=0,extra_pitch_damping=0;
 double get(const std::string&k,double fallback=0)const{auto it=v.find(k);return it==v.end()?fallback:it->second[0];}
 double opt(const std::string&k,double fallback)const{auto it=options.find(k);return it==options.end()?fallback:it->second;}
 A4 arr(const std::string&k,double fallback=0)const{A4 a;auto it=v.find(k);for(int i=0;i<4;i++)a[i]=it==v.end()?fallback:it->second[it->second.size()==1?0:i];return a;}
 void prepare(){
  ms=get("ms");m=get("m");g=get("g");Ixx=get("Ixx");Iyy=get("Iyy");Iz=get("Iz");L=get("L");lf=get("lf");lr=get("lr");tf=get("tf");tr=get("tr");h=get("h_cg_sprung");
  Kaf=get("Karb_f");Kar=get("Karb_r");Caf=get("Carb_f");Car=get("Carb_r");KP=get("speed_KP");KI=get("speed_KI");KP3=get("speed_KP3");power=get("max_power_W");rear_drive=get("rear_drive_fraction");rho=get("rho_air");area=get("aero_area_m2");Cx=get("aero_Cx");steering_ratio=get("steering_ratio");gear_ratio=get("gear_rot_ratio");
  ux_target=opt("speed_kmh",90)/3.6;mu=opt("mu",.6);steer=opt("steer_deg",2.5);
  mu_u=arr("mu_u");Iw=arr("Iw");wheelheight=arr("wheel_center_height_m");camberstatic=arr("camber_static_rad");toestatic=arr("toe_static_rad");caster=arr("caster_static_rad");kpi=arr("kingpin_inclination_rad");
  CT_FX=arr("CT_FX");CS_FY=arr("CS_FY");CS_MZ=arr("CS_MZ");CC_FX=arr("CC_FX");CI_FY=arr("CI_FY");CI_MZ=arr("CI_MZ");
  deflect=opt("tire_deformation_states",1);ldeflect=opt("tire_longitudinal_deformation",0);source_drive=opt("source_drive_model",1);rollgeom=opt("roll_geometry_balance",1);spatial=opt("spatial_wheel_velocity",1);compliance_body=opt("compliance_body_frame",1);steering_camber_on=opt("steering_axis_camber",1);full_orientation=opt("full_wheel_orientation",0);heading=opt("heading_frame_translation",1);coupled=opt("coupled_sprung_reference",1);coupled_pitch=opt("coupled_pitch_reference",0);support_projection=opt("support_force_projection",0);wheel_spin_gyro=opt("wheel_spin_gyro",0);vertical=opt("vertical_model",1);jacking=opt("geometric_jacking",1);rolling=opt("rolling_pitch_moment",1);compliance=opt("include_compliance",1);angular=int(opt("angular_model",2));ddelay=opt("damper_delay_s",.002);dsample=opt("damper_sample_dt",.001);
  extra_roll_damping=opt("extra_roll_damping_Nms",get("extra_roll_damping_Nms",0));extra_pitch_damping=opt("extra_pitch_damping_Nms",get("extra_pitch_damping_Nms",0));
  corrected_cc_fx_sign=opt("corrected_cc_fx_sign",0)!=0;
  if(!std::isfinite(extra_roll_damping)||!std::isfinite(extra_pitch_damping)||extra_roll_damping<0||extra_pitch_damping<0)throw std::runtime_error("Effective extra damping must be finite and nonnegative");
  xw={lf,lf,-lr,-lr};yw={tf/2,-tf/2,tr/2,-tr/2};Iz_total=Iz;auto modes=arr("spring_modes",2);
  for(int i=0;i<4;i++){
   tires[i].mf62_correct_trail=opt("mf62_zero_camber_trail",0)!=0;
   Fs0[i]=ms*g*(i<2?lr:lf)/(2*L);Fz0[i]=Fs0[i]+mu_u[i]*g;Iz_total+=mu_u[i]*(sq(xw[i])+sq(yw[i]));
   kt[i]=tires[i].VERTICAL_STIFFNESS;ct[i]=tires[i].VERTICAL_DAMPING;pressure[i]=tires[i].INFLPRES;
   const Table&t=tabs[SPRING_COMP][i];Table inverse;inverse.setup(t.y.data(),t.x.data(),int(t.x.size()),false);comp0[i]=inverse.value(Fs0[i]);j0[i]=comp0[i];
   static const double fr[3]={22161,30790,36086},rr[3]={23632,30790,35007};int mode=int(modes[i])-1;if(mode<0||mode>2)throw std::runtime_error("Invalid spring mode");springrate[i]=i<2?fr[mode]:rr[mode];
   Re0[i]=effective_radius(Fz0[i],ux_target/tires[i].UNLOADED_RADIUS,tires[i]);
  }
 }
 static double effective_radius(double fz,double om,const Tire&t){double r0=t.UNLOADED_RADIUS,f0=t.FNOMIN;return std::max(r0*(t.Q_RE0+t.Q_V1*sq(om*r0/t.LONGVL))-f0/t.VERTICAL_STIFFNESS*(t.DREFF*std::atan(t.BREFF*fz/f0)+t.FREFF*fz/f0),.1*r0);}
 static double loaded_radius(double fz,double om,double pres,const Tire&t){double r0=t.UNLOADED_RADIUS,fn=t.FNOMIN,v=std::abs(om)*r0/t.LONGVL,q1=std::sqrt(sq(t.VERTICAL_STIFFNESS*r0/fn)-4*t.Q_FZ2),normalizer=fn*(1+t.PFZ1*(pres-t.NOMPRES)/t.NOMPRES)*(1+t.Q_V2*v),z=std::max(fz,0.0)/normalizer;double rho=r0*2*z/(q1+std::sqrt(q1*q1+4*t.Q_FZ2*z));return std::max(r0*(t.Q_RE0+t.Q_V1*v*v)-rho,.1*r0);}
 void angles(const State&x,double&pd,double&td,double&yd)const{double p=x[10],q=x[11],r=x[1],ph=x[3],th=x[4];if(!angular){pd=p;td=q;yd=r;return;}pd=p+std::tan(th)*(q*std::sin(ph)+r*std::cos(ph));td=q*std::cos(ph)-r*std::sin(ph);yd=(q*std::sin(ph)+r*std::cos(ph))/std::cos(th);}
 void loads(const State&x,A4&fz,A4&raw)const{
  for(int i=0;i<4;i++){if(vertical){const Tire&t=tires[i];double r0=t.UNLOADED_RADIUS,fn=t.FNOMIN,j=j0[i]+x[5+i]-(x[2]+yw[i]*x[3]-xw[i]*x[4]),camber=-side[i]*(camberstatic[i]+tabs[CAMBER][i].value(j))+x[3],cs=std::max(std::cos(camber),.5),q1=std::sqrt(sq(t.VERTICAL_STIFFNESS*r0/fn)-4*t.Q_FZ2),speed=std::abs(x[24+i])*r0/t.LONGVL,rfree=r0*(t.Q_RE0+t.Q_V1*speed*speed),rho0=std::max(rfree-(wheelheight[i]+x[5+i])/cs,0.0)/r0,dp=(pressure[i]-t.NOMPRES)/t.NOMPRES;raw[i]=fn*(1+t.PFZ1*dp)*(1+t.Q_V2*speed)*(q1*rho0+t.Q_FZ2*rho0*rho0)-ct[i]*x[12+i]/cs;}else raw[i]=Fz0[i]-kt[i]*x[5+i]-ct[i]*x[12+i];fz[i]=std::max(raw[i],0.0);}
 }
 TF state_force(double k,double a,double fz,double ga,double uw,int i)const{
  if(deflect){auto sd=relaxation(fz,ga,uw,pressure[i],tires[i],mu);if(ldeflect)k/=std::max(sd[0],.001);a=std::atan(a/std::max(sd[1],.001));}
  return tire_force(k,a,fz,ga,uw,pressure[i],tires[i],mu);
 }
 void wheel_vel(const State&x,const A4&xx,const A4&yy,const A4&j,A4&u,A4&v)const{
  if(!spatial){for(int i=0;i<4;i++){u[i]=x[32]-yy[i]*x[1];v[i]=x[0]+xx[i]*x[1];}return;}
  double pd,td,yd;angles(x,pd,td,yd);double cp=std::cos(x[3]),sp=std::sin(x[3]),ct0=std::cos(x[4]),st=std::sin(x[4]);
  for(int i=0;i<4;i++){double jr=x[12+i]-(x[9]+yw[i]*pd-xw[i]*td),z=wheelheight[i]-h+j[i]-j0[i],sx=tabs[SUSP_X][i].slope(j[i]),sy=-side[i]*tabs[SUSP_LAT][i].slope(j[i]);double a=x[11]*z-x[1]*yy[i]+sx*jr,b=x[1]*xx[i]-x[10]*z+sy*jr,d=x[10]*yy[i]-x[11]*xx[i]+jr;u[i]=x[32]+ct0*a+st*sp*b+st*cp*d;v[i]=x[0]+cp*b-sp*d;}
 }
 double steer_camber(double target,double toe,double gamma,int i)const{
  double C=caster[i],K=kpi[i],axis[3]={-std::sin(C),-side[i]*std::sin(K)*std::cos(C),std::cos(K)*std::cos(C)},n[3]={-std::sin(toe)*std::cos(gamma),std::cos(toe)*std::cos(gamma),std::sin(gamma)},cr[3]={axis[1]*n[2]-axis[2]*n[1],axis[2]*n[0]-axis[0]*n[2],axis[0]*n[1]-axis[1]*n[0]},dot=axis[0]*n[0]+axis[1]*n[1]+axis[2]*n[2],a=(target-toe)/axis[2];
  for(int it=0;it<4;it++){double co=std::cos(a),si=std::sin(a),v[3],d[3];for(int k=0;k<3;k++){v[k]=n[k]*co+cr[k]*si+axis[k]*dot*(1-co);d[k]=-n[k]*si+cr[k]*co+axis[k]*dot*si;}double head=std::atan2(-v[0],v[1]),der=(-v[1]*d[0]+v[0]*d[1])/(v[0]*v[0]+v[1]*v[1]);a-=std::atan2(std::sin(head-target),std::cos(head-target))/der;}
  double vz=n[2]*std::cos(a)+cr[2]*std::sin(a)+axis[2]*dot*(1-std::cos(a));return std::asin(clamp(vz,-1,1))-gamma;
 }
 void geometry(const State&x,const A4&fz,A4&delta,A4&gamma,A4&xx,A4&yy,A4&j)const{
  A4 toe_unsteered,gamma_unsteered;double gear=steer*steering_ratio*PI/180*gear_ratio;
  for(int i=0;i<4;i++){j[i]=j0[i]+x[5+i]-(x[2]+yw[i]*x[3]-xw[i]*x[4]);double bt=toestatic[i]+tabs[TOE][i].value(j[i]),bc=camberstatic[i]+tabs[CAMBER][i].value(j[i]),command=i<2?tabs[STEERING][i].value(gear,true):0;delta[i]=command-side[i]*bt;toe_unsteered[i]=-side[i]*bt;gamma[i]=-side[i]*bc+x[3];xx[i]=xw[i]+tabs[SUSP_X][i].value(j[i]);yy[i]=yw[i]-side[i]*tabs[SUSP_LAT][i].value(j[i]);}
  gamma_unsteered=gamma;if(steering_camber_on)for(int i=0;i<2;i++)gamma[i]+=steer_camber(delta[i],toe_unsteered[i],gamma_unsteered[i]-x[3],i);
  if(compliance){A4 base_delta=delta,base_gamma=gamma_unsteered,ub,vb;wheel_vel(x,xx,yy,j,ub,vb);double cp=std::cos(x[3]),sp=std::sin(x[3]),ct0=std::cos(x[4]),st=std::sin(x[4]);
   for(int it=0;it<2;it++)for(int i=0;i<4;i++){double cd=std::cos(delta[i]),sd=std::sin(delta[i]),uw=cd*ub[i]+sd*vb[i];TF f=state_force(x[16+i],x[20+i],fz[i],gamma[i],uw,i);double fx=f.Fx,fy=f.Fy,mz=f.Mz;
    if(compliance_body){double fxr=fx*cd-fy*sd,fyr=fx*sd+fy*cd;fx=ct0*fxr-st*fz[i];fy=sp*st*fxr+cp*fyr+sp*ct0*fz[i];mz=-cp*st*(f.Mx*cd-f.My*sd)-sp*(f.Mx*sd+f.My*cd)+cp*ct0*mz;}
    delta[i]=base_delta[i]-side[i]*CT_FX[i]*fx+CS_FY[i]*fy+CS_MZ[i]*mz;gamma[i]=base_gamma[i]+(corrected_cc_fx_sign?-1:1)*side[i]*CC_FX[i]*fx+CI_FY[i]*fy+CI_MZ[i]*mz;if(steering_camber_on&&i<2)gamma[i]+=steer_camber(delta[i],toe_unsteered[i],base_gamma[i]-x[3],i);
   }
  }
  if(full_orientation)for(int i=0;i<4;i++){double dive=tabs[DIVE][i].value(j[i]),lg=gamma[i]-x[3],d=delta[i],nx=std::cos(d)*std::sin(dive)*std::sin(lg)-std::sin(d)*std::cos(lg),ny=std::sin(d)*std::sin(dive)*std::sin(lg)+std::cos(d)*std::cos(lg),nz=std::cos(dive)*std::sin(lg),cp=std::cos(x[3]),sp=std::sin(x[3]),ct0=std::cos(x[4]),st=std::sin(x[4]),wx=ct0*nx+st*sp*ny+st*cp*nz,wy=cp*ny-sp*nz,wz=-st*nx+ct0*sp*ny+ct0*cp*nz;delta[i]=std::atan2(-wx,wy);gamma[i]=std::asin(clamp(wz,-1,1));}
 }
 void slips(const State&x,const A4&fz,const A4&delta,const A4&xx,const A4&yy,const A4&j,A4&k,A4&a,A4&uw,A4&vw,A4&re)const{A4 ub,vb;wheel_vel(x,xx,yy,j,ub,vb);for(int i=0;i<4;i++){uw[i]=std::cos(delta[i])*ub[i]+std::sin(delta[i])*vb[i];vw[i]=-std::sin(delta[i])*ub[i]+std::cos(delta[i])*vb[i];a[i]=std::atan2(vw[i],uw[i]);re[i]=effective_radius(fz[i],x[24+i],tires[i]);k[i]=(re[i]*x[24+i]-uw[i])/std::max(std::abs(uw[i]),.5);}}
 void suspension(const State&x,const A4&j,A4&fs,A4&fd,A4&pure,const A4*over=nullptr)const{
  double pd,td,yd;angles(x,pd,td,yd);for(int i=0;i<4;i++){double rate=x[12+i]-(x[9]+yw[i]*pd-xw[i]*td),comp=comp0[i]+j[i]-j0[i],f=tabs[rate<0?SPRING_EXT:SPRING_COMP][i].value(comp);f+=(springrate[i]-30790)*comp;f+=tabs[JOUNCE_STOP][i].value(j[i])+tabs[REBOUND_STOP][i].value(j[i]);fd[i]=over?(*over)[i]:damp[i].value(1000*rate,true);pure[i]=f;fs[i]=f+fd[i];}
  double frel=x[3]-(x[5]-x[6])/tf,rrel=x[3]-(x[7]-x[8])/tr,frate=x[10]-(x[12]-x[13])/tf,rrate=x[10]-(x[14]-x[15])/tr,mf=-Kaf*frel-Caf*frate,mr=-Kar*rrel-Car*rrate;fs[0]+=mf/tf;fs[1]-=mf/tf;fs[2]+=mr/tr;fs[3]-=mr/tr;
 }
 struct Obs{A4 raw,fz,fx,fy,mz,my,mx,delta,gamma,jounce,spring,damper,jacking;double ay,ax,throttle;};
 void rhs(const State&x,State&dx,Obs*out=nullptr,const A4*override=nullptr)const;
 void trimstate(const double*y,State&x)const;
};

void Model::rhs(const State&x,State&dx,Obs*out,const A4*override)const{
 A4 fz,raw,de,ga,xx,yy,j,ktarget,atarget,uw,vw,re,fx,fy,mz,my,mx,sx,sy,fxb,fyb,fs,fd,pure,J,support,dFs;
 loads(x,fz,raw);geometry(x,fz,de,ga,xx,yy,j);slips(x,fz,de,xx,yy,j,ktarget,atarget,uw,vw,re);
 double sumfy=0,sumfx=0,sum_mz=0;
 for(int i=0;i<4;i++){TF f=state_force(x[16+i],x[20+i],fz[i],ga[i],uw[i],i);fx[i]=f.Fx;fy[i]=f.Fy;mz[i]=f.Mz;my[i]=f.My;mx[i]=f.Mx;sx[i]=std::max(f.sx,.001);sy[i]=std::max(f.sy,.001);fxb[i]=fx[i]*std::cos(de[i])-fy[i]*std::sin(de[i]);fyb[i]=fx[i]*std::sin(de[i])+fy[i]*std::cos(de[i]);sumfy+=fyb[i];sumfx+=fxb[i];sum_mz+=xx[i]*fyb[i]-yy[i]*fxb[i]+mz[i];}
 double spinMx=0,spinMy=0,spinMz=0;
 if(wheel_spin_gyro){double Hx=0,Hy=0,Hz=0;for(int i=0;i<4;i++){double angular_momentum=Iw[i]*x[24+i];Hx-=angular_momentum*std::sin(de[i])*std::cos(ga[i]);Hy+=angular_momentum*std::cos(de[i])*std::cos(ga[i]);Hz+=angular_momentum*std::sin(ga[i]);}spinMx=x[1]*Hy-x[11]*Hz;spinMy=x[10]*Hz-x[1]*Hx;spinMz=x[11]*Hx-x[10]*Hy;}
 if(extra_roll_damping>0||extra_pitch_damping>0){
  // Optional effective reduced-model damping. Euler angle rates vanish
  // in a steady turn; body p/q generally do not. Coefficients are N*m*s/rad.
  double roll_rate,pitch_rate,yaw_rate;angles(x,roll_rate,pitch_rate,yaw_rate);
  spinMx-=extra_roll_damping*roll_rate;spinMy-=extra_pitch_damping*pitch_rate;
 }
 double ay=sumfy/m,drag=.5*rho*area*Cx*x[32]*std::abs(x[32]),ax=(sumfx-drag)/m;
 suspension(x,j,fs,fd,pure,override);
 for(int i=0;i<4;i++){J[i]=0;if(jacking){double dxj=tabs[SUSP_X][i].slope(j[i]),dyj=-side[i]*tabs[SUSP_LAT][i].slope(j[i]),dgj=-side[i]*tabs[CAMBER][i].slope(j[i]),zw=wheelheight[i]+x[5+i];J[i]=-fxb[i]*dxj-fyb[i]*(dyj+zw*dgj);if(rollgeom){double residual=(mx[i]+zw*std::tan(ga[i])*fz[i])*std::cos(de[i])-my[i]*std::sin(de[i]);J[i]-=residual*dgj;}}support[i]=fs[i]+J[i];if(support_projection)support[i]=(support[i]+std::sin(x[3])*fyb[i]-std::cos(x[3])*std::sin(x[4])*fxb[i])/(std::cos(x[3])*std::cos(x[4]));dFs[i]=support[i]-Fs0[i];}
 double pd,td,yd;angles(x,pd,td,yd);double frame_yaw=heading?yd:x[1];dx.fill(0);dx[0]=ay-x[32]*frame_yaw;dx[1]=(sum_mz+spinMz)/Iz_total;for(int i=2;i<=8;i++)dx[i]=x[i+7];dx[3]=pd;dx[4]=td;
 double sum_dFs=0,mr=0,mp=0;for(int i=0;i<4;i++){sum_dFs+=dFs[i];mr+=yw[i]*dFs[i];mp-=xw[i]*dFs[i];}dx[9]=sum_dFs/ms;dx[10]=(mr+ms*h*ay+spinMx)/Ixx;dx[11]=(mp-ms*h*ax+spinMy)/Iyy;
 if(rollgeom){double levermoment=0,tiremoment=0,lateral=ms*(h+x[2]);for(int i=0;i<4;i++){double zr=wheelheight[i]-h+j[i]-j0[i],lever=yy[i]*std::cos(x[3])-zr*std::sin(x[3]);levermoment+=lever*support[i]-yw[i]*Fs0[i];tiremoment+=(mx[i]+(wheelheight[i]+x[5+i])*std::tan(ga[i])*fz[i])*std::cos(de[i])-my[i]*std::sin(de[i]);lateral+=mu_u[i]*(wheelheight[i]+x[5+i]);}dx[10]=(levermoment+lateral*ay+tiremoment+spinMx)/Ixx;}
 if(rolling)for(int i=0;i<4;i++)dx[11]+=my[i]*std::cos(de[i])/Iyy;
 if(angular==2){dx[10]+=(Iyy-Iz)/Ixx*x[11]*x[1];dx[11]+=(Iz-Ixx)/Iyy*x[1]*x[10];dx[1]+=(Ixx-Iyy)/Iz_total*x[10]*x[11];}
 if(coupled){double Az=0,Ax=0,Ay=0,Z2=0,Zx=0,Zy=0,Mr=0,Mzt=0,inertia=Iz;
  for(int i=0;i<4;i++){double zr=wheelheight[i]-h+j[i]-j0[i];Az+=mu_u[i]*zr;Ax+=mu_u[i]*xx[i];Ay+=mu_u[i]*yy[i];Z2+=mu_u[i]*zr*zr;Zx+=mu_u[i]*zr*xx[i];Zy+=mu_u[i]*zr*yy[i];double lever=yy[i]*std::cos(x[3])-zr*std::sin(x[3]);Mr+=lever*support[i]-yw[i]*Fs0[i];Mr+=(mx[i]+(wheelheight[i]+x[5+i])*std::tan(ga[i])*fz[i])*std::cos(de[i])-my[i]*std::sin(de[i]);Mzt+=xx[i]*fyb[i]-yy[i]*fxb[i]+mz[i]-(wheelheight[i]+x[5+i])*std::tan(ga[i])*fx[i];inertia+=mu_u[i]*(xx[i]*xx[i]+yy[i]*yy[i]);}
  double fy_rhs=sumfy+Ay*frame_yaw*frame_yaw;Mr+=(h+x[2])*sumfy-Zy*frame_yaw*frame_yaw+spinMx;Mzt+=spinMz;if(angular==2){Mr+=(Iyy-Iz)*x[11]*x[1];Mzt+=(Ixx-Iyy)*x[10]*x[11];}
  double ar=Ixx+Z2-Az*Az/m,coupling=-Zx+Az*Ax/m,iz=inertia-Ax*Ax/m,br=Mr+Az*fy_rhs/m,bz=Mzt-Ax*fy_rhs/m,det=ar*iz-coupling*coupling,pdot=(br*iz-coupling*bz)/det,rdot=(ar*bz-coupling*br)/det;
  ay=(fy_rhs+Az*pdot-Ax*rdot)/m;dx[0]=ay-x[32]*frame_yaw;dx[1]=rdot;dx[10]=pdot;
 }
 if(coupled_pitch){
  // Optional symmetric sprung-reference longitudinal/pitch inertia block.
  // This extension is OFF in the faithful V3 baseline.
  double Az=0,Ax=0,Z2=0,Zx=0,Mp=0;
  for(int i=0;i<4;i++){double zr=wheelheight[i]-h+j[i]-j0[i];Az+=mu_u[i]*zr;Ax+=mu_u[i]*xx[i];Z2+=mu_u[i]*zr*zr;Zx+=mu_u[i]*zr*xx[i];Mp-=xw[i]*dFs[i];if(rolling)Mp+=my[i]*std::cos(de[i]);}
  double fx_rhs=sumfx-drag+Ax*frame_yaw*frame_yaw;
  Mp-=(h+x[2])*sumfx;Mp+=Zx*frame_yaw*frame_yaw+spinMy;
  if(angular==2)Mp+=(Iz-Ixx)*x[1]*x[10];
  double qdot=(Mp-Az*fx_rhs/m)/(Iyy+Z2-Az*Az/m);
  ax=(fx_rhs-Az*qdot)/m;dx[11]=qdot;
 }
 for(int i=0;i<4;i++){dx[12+i]=(raw[i]-Fz0[i]-dFs[i])/mu_u[i];double speed=std::max(std::abs(uw[i]),.5);if(deflect){dx[16+i]=ldeflect?speed*(ktarget[i]-x[16+i]/sx[i]):speed/sx[i]*(ktarget[i]-x[16+i]);dx[20+i]=speed*(std::tan(atarget[i])-x[20+i]/sy[i]);}else{dx[16+i]=speed/sx[i]*(ktarget[i]-x[16+i]);dx[20+i]=speed/sy[i]*(atarget[i]-x[20+i]);}}
 double error=ux_target-x[32],demand=KP*error+KP3*error*error*error+KI*x[33],throttle=0,rawforce=0,limit=0;
 if(source_drive){rawforce=m*g*demand;limit=power/std::max(std::abs(x[32]),1.0);double drive=clamp(rawforce,0,limit);throttle=drive/limit;A4 rl;for(int i=0;i<4;i++)rl[i]=loaded_radius(fz[i],x[24+i],pressure[i],tires[i]);double ft=(1-rear_drive)*drive/(1/rl[0]+1/rl[1]),rt=rear_drive*drive/(1/rl[2]+1/rl[3]);for(int i=0;i<4;i++)dx[24+i]=((i<2?ft:rt)-rl[i]*fx[i]+my[i]*std::cos(ga[i])+mz[i]*std::sin(ga[i]))/Iw[i];}
 else{throttle=clamp(demand,0,1);double drive=throttle*power/std::max(std::abs(x[32]),1.0);for(int i=0;i<4;i++){double torque=(i<2?1-rear_drive:rear_drive)/2*drive*re[i];dx[24+i]=(torque-re[i]*fx[i]+my[i])/Iw[i];}}
 dx[28]=ay-x[28];dx[29]=yd;dx[30]=x[32]*std::cos(x[29])-x[0]*std::sin(x[29]);dx[31]=x[32]*std::sin(x[29])+x[0]*std::cos(x[29]);dx[32]=ax+frame_yaw*x[0];dx[33]=error;
 bool windup=source_drive?((rawforce<=0&&error<0)||(rawforce>=limit&&error>0)):((demand<=0&&error<0)||(demand>=1&&error>0));if(windup)dx[33]=0;
 if(out){out->raw=raw;out->fz=fz;out->fx=fx;out->fy=fy;out->mz=mz;out->my=my;out->mx=mx;out->delta=de;out->gamma=ga;out->jounce=j;out->spring=pure;out->damper=fd;out->jacking=J;out->ay=ay;out->ax=ax;out->throttle=throttle;}
}

void Model::trimstate(const double*y,State&x)const{
 if(deflect){Model q=*this;q.deflect=false;q.trimstate(y,x);A4 fz,raw,de,ga,xx,yy,j,ub,vb; q.loads(x,fz,raw);q.geometry(x,fz,de,ga,xx,yy,j);q.wheel_vel(x,xx,yy,j,ub,vb);
  for(int i=0;i<4;i++){double uw=std::cos(de[i])*ub[i]+std::sin(de[i])*vb[i];TF d=tire_force(y[9+i],y[13+i],fz[i],ga[i],uw,pressure[i],tires[i],mu);x[16+i]=ldeflect?d.sx*y[9+i]:y[9+i];x[20+i]=d.sy*std::tan(y[13+i]);}
  for(int it=0;it<12;it++){State old=x;loads(x,fz,raw);geometry(x,fz,de,ga,xx,yy,j);wheel_vel(x,xx,yy,j,ub,vb);for(int i=0;i<4;i++){double uw=std::cos(de[i])*ub[i]+std::sin(de[i])*vb[i];TF d=tire_force(y[9+i],y[13+i],fz[i],ga[i],uw,pressure[i],tires[i],mu);x[16+i]=ldeflect?d.sx*y[9+i]:y[9+i];x[20+i]=d.sy*std::tan(y[13+i]);double re=effective_radius(fz[i],x[24+i],tires[i]);x[24+i]=uw*(1+y[9+i])/re;}double change=0;for(int k=16;k<28;k++)change=std::max(change,std::abs(x[k]-old[k]));if(change<1e-11)break;}return;
 }
 x.fill(0);for(int i=0;i<9;i++)x[i]=y[i];for(int i=0;i<4;i++){x[16+i]=y[9+i];x[20+i]=y[13+i];}x[32]=ux_target;x[33]=y[17];if(angular){x[10]=-x[1]*std::tan(x[4])/std::cos(x[3]);x[11]=x[1]*std::tan(x[3]);}
 A4 fz,raw,de,ga,xx,yy,j,ub,vb;loads(x,fz,raw);geometry(x,fz,de,ga,xx,yy,j);wheel_vel(x,xx,yy,j,ub,vb);for(int i=0;i<4;i++)x[24+i]=(std::cos(de[i])*ub[i]+std::sin(de[i])*vb[i])*(1+x[16+i])/Re0[i];
 for(int it=0;it<14;it++){if(vertical){loads(x,fz,raw);geometry(x,fz,de,ga,xx,yy,j);wheel_vel(x,xx,yy,j,ub,vb);}double change=0;for(int i=0;i<4;i++){double old=x[24+i],re=effective_radius(fz[i],old,tires[i]);x[24+i]=(std::cos(de[i])*ub[i]+std::sin(de[i])*vb[i])*(1+x[16+i])/re;change=std::max(change,std::abs(x[24+i]-old));}if(change<1e-11)break;}
 double pd,td,yd;angles(x,pd,td,yd);x[28]=ux_target*(heading?yd:x[1]);
}

static thread_local std::string last_error;
extern "C" {
const char*hmc_error(){return last_error.c_str();}
void*hmc_create(){try{return new Model();}catch(const std::exception&e){last_error=e.what();return nullptr;}}
void hmc_destroy(void*p){delete static_cast<Model*>(p);}
int hmc_set_vehicle(void*p,const char*k,const double*values,int n){try{static_cast<Model*>(p)->v[k]=std::vector<double>(values,values+n);return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_set_tire(void*p,int i,const char*k,double v){try{static_cast<Model*>(p)->tires.at(i).set(k,v);return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_set_option(void*p,const char*k,double v){static_cast<Model*>(p)->options[k]=v;return 0;}
int hmc_set_table(void*p,const char*g,int corner,const double*x,const double*y,int n,int spline){try{int group=-1;for(int i=0;i<NGROUPS;i++)if(std::strcmp(g,groupnames[i])==0)group=i;if(group<0)throw std::runtime_error("Invalid table group");static_cast<Model*>(p)->tabs.at(group).at(corner).setup(x,y,n,spline);return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_set_damper(void*p,int corner,const double*x,const double*y,int n){try{static_cast<Model*>(p)->damp.at(corner).setup(x,y,n,false);return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_prepare(void*p){try{static_cast<Model*>(p)->prepare();return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_tire_force(void*p,int i,double kappa,double alpha,double fz,double gamma,double vx,double*out){try{const Model&q=*static_cast<Model*>(p);const Tire&t=q.tires.at(i);TF o=tire_force(kappa,alpha,fz,gamma,vx,q.pressure.at(i),t,q.mu);out[0]=o.Fx;out[1]=o.Fy;out[2]=o.Mz;out[3]=o.My;out[4]=o.Mx;out[5]=o.sx;out[6]=o.sy;return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_trimstate(void*p,const double*y,double*out){try{State x;static_cast<Model*>(p)->trimstate(y,x);std::copy(x.begin(),x.end(),out);return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_rhs(void*p,const double*xx,double*out,double*obs,const double*fd){try{State x,d;std::copy(xx,xx+34,x.begin());A4 over;if(fd)std::copy(fd,fd+4,over.begin());Model::Obs o;static_cast<Model*>(p)->rhs(x,d,obs?&o:nullptr,fd?&over:nullptr);std::copy(d.begin(),d.end(),out);if(obs){obs[0]=o.ay;obs[1]=o.ax;obs[2]=o.throttle;int k=3;for(auto a:{o.raw,o.fx,o.fy,o.mz,o.my,o.mx,o.delta,o.gamma,o.jounce,o.spring,o.damper,o.jacking})for(auto z:a)obs[k++]=z;}return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
int hmc_trajectory(void*p,const double*xx,const double*trim,double dt,int steps,double*out,double*obsout){try{const Model&q=*static_cast<Model*>(p);State x,a,b,d,e,temp;std::copy(xx,xx+34,x.begin());A4 held;std::vector<A4>history;int pos=0,sample_steps=1,lag=0;if(q.ddelay>0){sample_steps=std::lround(q.dsample/dt);lag=std::lround(q.ddelay/q.dsample);if(sample_steps<1||lag<1||std::abs(sample_steps*dt-q.dsample)>1e-10||std::abs(lag*q.dsample-q.ddelay)>1e-10)throw std::runtime_error("CDC times must lie on integration lattice");State past=x;if(trim)std::copy(trim,trim+34,past.begin());Model::Obs o;q.rhs(past,a,&o);held=o.damper;history.assign(lag,held);}
 for(int k=0;k<=steps;k++){if(lag&&k%sample_steps==0){Model::Obs o;q.rhs(x,a,&o);held=history[pos];history[pos]=o.damper;pos=(pos+1)%lag;}Model::Obs o;q.rhs(x,a,&o,lag?&held:nullptr);for(int j=0;j<34;j++){if(!std::isfinite(x[j])||!std::isfinite(a[j]))throw std::runtime_error("Nonfinite trajectory state or RHS");out[k*34+j]=x[j];}if(obsout){obsout[k*15]=o.ay;obsout[k*15+1]=o.ax;obsout[k*15+2]=o.throttle;for(int i=0;i<4;i++){obsout[k*15+3+i]=o.raw[i];obsout[k*15+7+i]=o.spring[i];obsout[k*15+11+i]=o.damper[i];}}if(k==steps)break;
  for(int j=0;j<34;j++)temp[j]=x[j]+.5*dt*a[j];q.rhs(temp,b,nullptr,lag?&held:nullptr);for(int j=0;j<34;j++)temp[j]=x[j]+.5*dt*b[j];q.rhs(temp,d,nullptr,lag?&held:nullptr);for(int j=0;j<34;j++)temp[j]=x[j]+dt*d[j];q.rhs(temp,e,nullptr,lag?&held:nullptr);for(int j=0;j<34;j++)x[j]+=dt/6*(a[j]+2*b[j]+2*d[j]+e[j]);
 }return 0;}catch(const std::exception&e){last_error=e.what();return -1;}}
}
