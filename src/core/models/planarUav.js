const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

export function wrapPlanarAngle(angle){
  let value=angle;
  while(value>Math.PI)value-=2*Math.PI;
  while(value<-Math.PI)value+=2*Math.PI;
  return value;
}

export function createPlanarUavConfig(cfg={}){
  const mass=cfg.mass??1.4;
  const gravity=cfg.gravity??9.81;
  return {
    dt:cfg.dt??0.01,
    mass,
    inertia:cfg.inertia??0.035,
    gravity,
    linearDragX:cfg.linearDragX??0.16,
    linearDragZ:cfg.linearDragZ??0.22,
    angularDamping:cfg.angularDamping??0.08,
    minThrust:cfg.minThrust??0,
    maxThrust:cfg.maxThrust??2.2*mass*gravity,
    maxTorque:cfg.maxTorque??0.7,
  };
}

export function planarUavDerivatives(state,input,cfg={}){
  const p=createPlanarUavConfig(cfg);
  const thrust=clamp(input.thrust,p.minThrust,p.maxThrust);
  const torque=clamp(input.torque,-p.maxTorque,p.maxTorque);
  return {
    xDot:state.vx,
    zDot:state.vz,
    thetaDot:state.q,
    vxDot:-(thrust/p.mass)*Math.sin(state.theta)-p.linearDragX*state.vx,
    vzDot:(thrust/p.mass)*Math.cos(state.theta)-p.gravity-p.linearDragZ*state.vz,
    qDot:torque/p.inertia-p.angularDamping*state.q,
  };
}

export function stepPlanarUav(state,input,cfg={}){
  const p=createPlanarUavConfig(cfg);
  const d=planarUavDerivatives(state,input,p);
  return {
    x:state.x+p.dt*d.xDot,
    z:state.z+p.dt*d.zDot,
    theta:wrapPlanarAngle(state.theta+p.dt*d.thetaDot),
    vx:state.vx+p.dt*d.vxDot,
    vz:state.vz+p.dt*d.vzDot,
    q:state.q+p.dt*d.qDot,
  };
}
