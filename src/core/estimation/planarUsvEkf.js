import {
  assertEstimatorContract,
  assertFiniteRecord,
} from '../../contracts/controlContracts.js';
import {
  createPlanarUsvConfig,
  stepPlanarUsv,
  wrapUsvAngle,
} from '../models/planarUsv.js';

const finite=(v,f=0)=>Number.isFinite(v)?v:f;
const eye=n=>Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?1:0));
const clone=A=>A.map(r=>[...r]);
function mm(A,B){return A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0)));}
function tr(A){return A[0].map((_,j)=>A.map(r=>r[j]));}
function add(A,B){return A.map((r,i)=>r.map((v,j)=>v+B[i][j]));}
function sym(P){return P.map((r,i)=>r.map((v,j)=>i===j?Math.max(0,finite(v)):0.5*(finite(v)+finite(P[j][i]))));}
function diag(values){return values.map((v,i)=>values.map((_,j)=>i===j?Math.max(1e-12,finite(v,1e-6)):0));}

export function createPlanarUsvEkf({
  cfg,
  initialState=[0,0.9,0,0,0,0],
  initialCovariance=[0.2,0.2,0.04,0.15,0.08,0.05],
  processCovariance=[1e-5,1e-5,1e-6,2e-4,2e-4,1e-4],
  measurementVariance=[0.12**2,0.12**2,0.02**2,0.08**2,0.05**2,0.025**2],
}){
  assertEstimatorContract({
    stateLength:6,
    initialState,
    initialCovariance,
    processCovariance,
    measurementVariance,
    label:'usvEkf',
  });
  const p=createPlanarUsvConfig(cfg);
  let x=[...initialState].map(finite);
  x[2]=wrapUsvAngle(x[2]);
  let P=diag(initialCovariance);
  const Q=diag(processCovariance);
  const R=[...measurementVariance].map(v=>Math.max(1e-12,finite(v,1e-3)));
  let diagnostics=null;

  function predict(input){
    assertFiniteRecord(input,['surgeForce','yawMoment'],'usvEkf.input');
    const previous=[...x];
    const state={
      x:previous[0],
      y:previous[1],
      psi:previous[2],
      u:previous[3],
      v:previous[4],
      r:previous[5],
    };
    const next=stepPlanarUsv(state,input,p);
    x=[next.x,next.y,next.psi,next.u,next.v,next.r];

    const dt=p.dt;
    const psi=previous[2];
    const u=previous[3];
    const v=previous[4];
    const r=previous[5];
    const c=Math.cos(psi);
    const s=Math.sin(psi);
    const F=eye(6);

    F[0][2]=dt*(-u*s-v*c);
    F[0][3]=dt*c;
    F[0][4]=-dt*s;

    F[1][2]=dt*(u*c-v*s);
    F[1][3]=dt*s;
    F[1][4]=dt*c;

    F[2][5]=dt;

    F[3][3]=1-dt*(p.surgeLinearDrag+2*p.surgeQuadraticDrag*Math.abs(u))/p.mass;
    F[3][4]=dt*r;
    F[3][5]=dt*v;

    F[4][3]=-dt*r;
    F[4][4]=1-dt*(p.swayLinearDrag+2*p.swayQuadraticDrag*Math.abs(v))/p.mass;
    F[4][5]=-dt*u;

    F[5][5]=1-dt*(p.yawLinearDrag+2*p.yawQuadraticDrag*Math.abs(r))/p.yawInertia;

    P=sym(add(mm(mm(F,P),tr(F)),Q));
    return getState();
  }

  function scalarUpdate(index,measurement,isAngle=false){
    const innovation=isAngle
      ?wrapUsvAngle(measurement-x[index])
      :measurement-x[index];
    const S=Math.max(1e-12,P[index][index]+R[index]);
    const K=P.map(row=>row[index]/S);
    x=x.map((value,i)=>value+K[i]*innovation);
    x[2]=wrapUsvAngle(x[2]);

    const M=eye(6);
    for(let i=0;i<6;i++) M[i][index]-=K[i];
    const KRKt=K.map(ki=>K.map(kj=>ki*R[index]*kj));
    P=sym(add(mm(mm(M,P),tr(M)),KRKt));
    return {innovation,innovationVariance:S,kalmanGain:[...K]};
  }

  function update(measurement){
    assertFiniteRecord(measurement,['x','y','psi','u','v','r'],'usvEkf.measurement');
    const entries=[
      measurement.x,
      measurement.y,
      measurement.psi,
      measurement.u,
      measurement.v,
      measurement.r,
    ];
    const innovations=[];
    const innovationVariances=[];
    for(let i=0;i<6;i++){
      const d=scalarUpdate(i,entries[i],i===2);
      innovations.push(d.innovation);
      innovationVariances.push(d.innovationVariance);
    }
    diagnostics={
      innovations,
      innovationVariances,
      covarianceTrace:P.reduce((sum,row,i)=>sum+row[i],0),
    };
    return {
      ...getState(),
      covariance:clone(P),
      diagnostics:{
        ...diagnostics,
        innovations:[...innovations],
        innovationVariances:[...innovationVariances],
      },
    };
  }

  function step(input,measurement){
    predict(input);
    return update(measurement);
  }

  function getState(){
    return {
      x:x[0],
      y:x[1],
      psi:x[2],
      u:x[3],
      v:x[4],
      r:x[5],
    };
  }

  return {
    predict,
    update,
    step,
    getState,
    getCovariance(){return clone(P);},
    getDiagnostics(){
      return diagnostics
        ?{
          ...diagnostics,
          innovations:[...diagnostics.innovations],
          innovationVariances:[...diagnostics.innovationVariances],
        }
        :null;
    },
  };
}
