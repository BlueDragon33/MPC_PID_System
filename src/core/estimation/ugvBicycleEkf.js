import {
  assertEstimatorContract,
  assertFiniteRecord,
} from '../../contracts/controlContracts.js';
import { createBicycleConfig, wrapAngle } from '../models/kinematicBicycle.js';

const finite=(v,f=0)=>Number.isFinite(v)?v:f;
const eye4=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
const clone=A=>A.map(r=>[...r]);
function mm(A,B){return A.map((r,i)=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));}
function tr(A){return A[0].map((_,j)=>A.map(r=>r[j]));}
function add(A,B){return A.map((r,i)=>r.map((v,j)=>v+B[i][j]));}
function sym(P){return P.map((r,i)=>r.map((v,j)=>i===j?Math.max(0,finite(v)):0.5*(finite(v)+finite(P[j][i]))));}
function diag(values){return values.map((v,i)=>values.map((_,j)=>i===j?Math.max(1e-12,finite(v,1e-6)):0));}

export function createUGVBicycleEkf({
  cfg,
  initialState = [0,0,0,0],
  initialCovariance = [0.3,0.3,0.08,0.2],
  processCovariance = [2e-4,2e-4,2e-5,3e-4],
  measurementVariance = [0.18**2,0.18**2,0.035**2,0.12**2],
}) {
  assertEstimatorContract({
    stateLength:4,
    initialState,
    initialCovariance,
    processCovariance,
    measurementVariance,
    label:'ugvEkf',
  });
  const p=createBicycleConfig(cfg);
  let x=[...initialState].map(finite);
  let P=diag(initialCovariance);
  const Q=diag(processCovariance);
  const R=[...measurementVariance].map(v=>Math.max(1e-12,finite(v,1e-3)));
  let diagnostics=null;

  function predict(input){
    assertFiniteRecord(input,['steer','accel'],'ugvEkf.input');
    const steer=Math.max(-p.maxSteer,Math.min(p.maxSteer,input.steer));
    const accel=Math.max(p.minAccel,Math.min(p.maxAccel,input.accel));
    const [px,py,yaw,v]=x; const dt=p.dt;
    x=[
      px+dt*v*Math.cos(yaw),
      py+dt*v*Math.sin(yaw),
      wrapAngle(yaw+dt*(v/p.wheelbase)*Math.tan(steer)),
      Math.max(p.minSpeed,Math.min(p.maxSpeed,v+dt*accel)),
    ];
    const F=[
      [1,0,-dt*v*Math.sin(yaw),dt*Math.cos(yaw)],
      [0,1, dt*v*Math.cos(yaw),dt*Math.sin(yaw)],
      [0,0,1,dt*Math.tan(steer)/p.wheelbase],
      [0,0,0,1],
    ];
    P=sym(add(mm(mm(F,P),tr(F)),Q));
    return getState();
  }

  function scalarUpdate(index,measurement,isAngle=false){
    const predicted=x[index];
    const innovation=isAngle?wrapAngle(measurement-predicted):measurement-predicted;
    const S=Math.max(1e-12,P[index][index]+R[index]);
    const K=P.map(row=>row[index]/S);
    x=x.map((value,i)=>value+K[i]*innovation);
    x[2]=wrapAngle(x[2]);

    const M=eye4();
    for(let i=0;i<4;i++) M[i][index]-=K[i];
    const KRKt=K.map((ki)=>K.map((kj)=>ki*R[index]*kj));
    P=sym(add(mm(mm(M,P),tr(M)),KRKt));
    return {innovation,innovationVariance:S,kalmanGain:[...K]};
  }

  function update(measurement){
    assertFiniteRecord(measurement,['x','y','yaw','v'],'ugvEkf.measurement');
    const entries=[measurement.x,measurement.y,measurement.yaw,measurement.v];
    const innovations=[]; const innovationVariances=[];
    for(let i=0;i<4;i++){
      const d=scalarUpdate(i,entries[i],i===2);
      innovations.push(d.innovation); innovationVariances.push(d.innovationVariance);
    }
    diagnostics={
      innovations,
      innovationVariances,
      covarianceTrace:P[0][0]+P[1][1]+P[2][2]+P[3][3],
    };
    return { ...getState(), covariance:clone(P), diagnostics:{...diagnostics,innovations:[...innovations],innovationVariances:[...innovationVariances]} };
  }

  function step(input,measurement){ predict(input); return update(measurement); }
  function getState(){return {x:x[0],y:x[1],yaw:x[2],v:x[3]};}

  return {
    predict,update,step,getState,
    getCovariance(){return clone(P);},
    getDiagnostics(){return diagnostics?{...diagnostics,innovations:[...diagnostics.innovations],innovationVariances:[...diagnostics.innovationVariances]}:null;},
  };
}
