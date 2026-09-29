const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

export function wrapUsvAngle(angle){
  let value=angle;
  while(value>Math.PI)value-=2*Math.PI;
  while(value<-Math.PI)value+=2*Math.PI;
  return value;
}

export function createPlanarUsvConfig(cfg={}){
  return {
    dt:cfg.dt??0.02,
    mass:cfg.mass??18,
    yawInertia:cfg.yawInertia??8,
    surgeLinearDrag:cfg.surgeLinearDrag??1.8,
    surgeQuadraticDrag:cfg.surgeQuadraticDrag??5.2,
    swayLinearDrag:cfg.swayLinearDrag??7.0,
    swayQuadraticDrag:cfg.swayQuadraticDrag??10.0,
    yawLinearDrag:cfg.yawLinearDrag??4.5,
    yawQuadraticDrag:cfg.yawQuadraticDrag??3.5,
    minSurgeForce:cfg.minSurgeForce??-30,
    maxSurgeForce:cfg.maxSurgeForce??45,
    maxYawMoment:cfg.maxYawMoment??18,
  };
}

export function planarUsvDerivatives(state,input,cfg={}){
  const p=createPlanarUsvConfig(cfg);
  const surgeForce=clamp(input.surgeForce,p.minSurgeForce,p.maxSurgeForce);
  const yawMoment=clamp(input.yawMoment,-p.maxYawMoment,p.maxYawMoment);
  const c=Math.cos(state.psi);
  const s=Math.sin(state.psi);
  const surgeDrag=p.surgeLinearDrag*state.u+p.surgeQuadraticDrag*state.u*Math.abs(state.u);
  const swayDrag=p.swayLinearDrag*state.v+p.swayQuadraticDrag*state.v*Math.abs(state.v);
  const yawDrag=p.yawLinearDrag*state.r+p.yawQuadraticDrag*state.r*Math.abs(state.r);

  return {
    xDot:state.u*c-state.v*s,
    yDot:state.u*s+state.v*c,
    psiDot:state.r,
    uDot:(surgeForce-surgeDrag)/p.mass+state.r*state.v,
    vDot:(-swayDrag)/p.mass-state.r*state.u,
    rDot:(yawMoment-yawDrag)/p.yawInertia,
  };
}

export function stepPlanarUsv(state,input,cfg={}){
  const p=createPlanarUsvConfig(cfg);
  const d=planarUsvDerivatives(state,input,p);
  return {
    x:state.x+p.dt*d.xDot,
    y:state.y+p.dt*d.yDot,
    psi:wrapUsvAngle(state.psi+p.dt*d.psiDot),
    u:state.u+p.dt*d.uDot,
    v:state.v+p.dt*d.vDot,
    r:state.r+p.dt*d.rDot,
  };
}
