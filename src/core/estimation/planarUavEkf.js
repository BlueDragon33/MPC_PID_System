import { createPlanarUavConfig, stepPlanarUav, wrapPlanarAngle } from '../models/planarUav.js';

const finite=(v,f=0)=>Number.isFinite(v)?v:f;
const eye=n=>Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?1:0));
const clone=A=>A.map(r=>[...r]);
function mm(A,B){return A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));}
function tr(A){return A[0].map((_,j)=>A.map(r=>r[j]));}
function add(A,B){return A.map((r,i)=>r.map((v,j)=>v+B[i][j]));}
function sym(P){return P.map((r,i)=>r.map((v,j)=>i===j?Math.max(0,finite(v)):0.5*(finite(v)+finite(P[j][i]))));}
function diag(values){return values.map((v,i)=>values.map((_,j)=>i===j?Math.max(1e-12,finite(v,1e-6)):0));}

export function createPlanarUavEkf({
  cfg,
  initialState=[0,1.55,0,0,0,0],
  initialCovariance=[0.12,0.10,0.03,0.15,0.15,0.08],
  processCovariance=[1e-5,1e-5,1e-6,3e-4,3e-4,4e-4],
  measurementVariance=[0.08**2,0.06**2,0.015**2,0.08**2,0.07**2,0.04**2],
}){
  const p=createPlanarUavConfig(cfg);
  let x=[...initialState].map(finite);
  x[2]=wrapPlanarAngle(x[2]);
  let P=diag(initialCovariance);
  const Q=diag(processCovariance);
  const R=[...measurementVariance].map(v=>Math.max(1e-12,finite(v,1e-3)));
  let diagnostics=null;

  function predict(input){
    const previous=[...x];
    const state={x:previous[0],z:previous[1],theta:previous[2],vx:previous[3],vz:previous[4],q:previous[5]};
    const next=stepPlanarUav(state,input,p);
    x=[next.x,next.z,next.theta,next.vx,next.vz,next.q];
    const thrust=Math.max(p.minThrust,Math.min(p.maxThrust,input.thrust));
    const dt=p.dt;
    const F=eye(6);
    F[0][3]=dt;
    F[1][4]=dt;
    F[2][5]=dt;
    F[3][2]=-dt*(thrust/p.mass)*Math.cos(previous[2]);
    F[3][3]=1-dt*p.linearDragX;
    F[4][2]=-dt*(thrust/p.mass)*Math.sin(previous[2]);
    F[4][4]=1-dt*p.linearDragZ;
    F[5][5]=1-dt*p.angularDamping;
    P=sym(add(mm(mm(F,P),tr(F)),Q));
    return getState();
  }

  function scalarUpdate(index,measurement,isAngle=false){
    const innovation=isAngle?wrapPlanarAngle(measurement-x[index]):measurement-x[index];
    const S=Math.max(1e-12,P[index][index]+R[index]);
    const K=P.map(row=>row[index]/S);
    x=x.map((value,i)=>value+K[i]*innovation);
    x[2]=wrapPlanarAngle(x[2]);
    const M=eye(6);
    for(let i=0;i<6;i++) M[i][index]-=K[i];
    const KRKt=K.map(ki=>K.map(kj=>ki*R[index]*kj));
    P=sym(add(mm(mm(M,P),tr(M)),KRKt));
    return {innovation,innovationVariance:S,kalmanGain:[...K]};
  }

  function update(measurement){
    const entries=[measurement.x,measurement.z,measurement.theta,measurement.vx,measurement.vz,measurement.q];
    const innovations=[]; const innovationVariances=[];
    for(let i=0;i<6;i++){
      const d=scalarUpdate(i,finite(entries[i]),i===2);
      innovations.push(d.innovation); innovationVariances.push(d.innovationVariance);
    }
    diagnostics={innovations,innovationVariances,covarianceTrace:P.reduce((s,row,i)=>s+row[i],0)};
    return {...getState(),covariance:clone(P),diagnostics:{...diagnostics,innovations:[...innovations],innovationVariances:[...innovationVariances]}};
  }

  function step(input,measurement){predict(input);return update(measurement);}
  function getState(){return {x:x[0],z:x[1],theta:x[2],vx:x[3],vz:x[4],q:x[5]};}
  return {predict,update,step,getState,getCovariance(){return clone(P);},getDiagnostics(){return diagnostics?{...diagnostics,innovations:[...diagnostics.innovations],innovationVariances:[...diagnostics.innovationVariances]}:null;}};
}
