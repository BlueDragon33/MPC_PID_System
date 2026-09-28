import { stepKinematicBicycle, wrapAngle } from '../models/kinematicBicycle.js';

const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

function uniqueSorted(values){
  return [...new Set(values.map(v=>Number(v.toFixed(12))))].sort((a,b)=>a-b);
}

export function createUGVPredictiveGovernor(cfg={}){
  const controlDt=cfg.dt??0.02;
  const predictionDt=cfg.predictionDt??0.08;
  const horizon=cfg.horizon??10;
  const maxSteer=cfg.maxSteer??0.55;
  const maxSteerRate=cfg.maxSteerRate??0.9;
  const minAccel=cfg.minAccel??-3;
  const maxAccel=cfg.maxAccel??2.5;
  const minSpeed=cfg.minSpeed??0;
  const maxSpeed=cfg.maxSpeed??8;
  const maxAbsCrossTrack=cfg.maxAbsCrossTrack??1.25;
  const maxAbsHeadingError=cfg.maxAbsHeadingError??0.8;
  const weights={
    crossTrack:cfg.qCrossTrack??8,
    heading:cfg.qHeading??3,
    speed:cfg.qSpeed??0.7,
    steer:cfg.rSteer??0.12,
    accel:cfg.rAccel??0.03,
    proposal:cfg.rProposal??0.22,
  };

  function evaluateCandidate(state,command,proposal,referenceAt,plantCfg){
    let s={...state};
    let cost=0;
    let predictedMaxCte=0;
    let predictedMaxHeading=0;
    let feasible=true;
    for(let i=0;i<horizon;i+=1){
      s=stepKinematicBicycle(s,command,{...plantCfg,dt:predictionDt});
      const ref=referenceAt(s.x);
      const cte=ref.y-s.y;
      const headingError=wrapAngle(ref.heading-s.yaw);
      const speedError=ref.speed-s.v;
      predictedMaxCte=Math.max(predictedMaxCte,Math.abs(cte));
      predictedMaxHeading=Math.max(predictedMaxHeading,Math.abs(headingError));
      if(
        Math.abs(cte)>maxAbsCrossTrack ||
        Math.abs(headingError)>maxAbsHeadingError ||
        s.v<minSpeed-1e-9 ||
        s.v>maxSpeed+1e-9
      ){
        feasible=false;
        break;
      }
      cost +=
        weights.crossTrack*cte*cte +
        weights.heading*headingError*headingError +
        weights.speed*speedError*speedError +
        weights.steer*command.steer*command.steer +
        weights.accel*command.accel*command.accel;
    }
    const ds=command.steer-proposal.steer;
    const da=command.accel-proposal.accel;
    cost += weights.proposal*(ds*ds+0.25*da*da);
    return {feasible,cost,predictedMaxCte,predictedMaxHeading};
  }

  return {
    select({state,proposal,previousCommand,referenceAt,plantCfg}){
      const start=performance.now();
      const maxSteerDelta=maxSteerRate*controlDt;
      const steerLo=Math.max(-maxSteer,previousCommand.steer-maxSteerDelta);
      const steerHi=Math.min(maxSteer,previousCommand.steer+maxSteerDelta);
      const proposedSteer=clamp(proposal.steer,steerLo,steerHi);
      const steeringCandidates=uniqueSorted([
        proposedSteer,
        steerLo,
        steerHi,
        clamp(previousCommand.steer-0.5*maxSteerDelta,steerLo,steerHi),
        clamp(previousCommand.steer+0.5*maxSteerDelta,steerLo,steerHi),
      ]);
      const accelCandidates=uniqueSorted([
        clamp(proposal.accel,minAccel,maxAccel),
        clamp(proposal.accel-0.45,minAccel,maxAccel),
        clamp(proposal.accel+0.45,minAccel,maxAccel),
      ]);

      let best=null;
      let feasibleCount=0;
      let candidateCount=0;
      for(const steer of steeringCandidates){
        for(const accel of accelCandidates){
          candidateCount+=1;
          const command={steer,accel};
          const evaluation=evaluateCandidate(state,command,proposal,referenceAt,plantCfg);
          if(!evaluation.feasible) continue;
          feasibleCount+=1;
          if(!best || evaluation.cost<best.cost){
            best={...command,...evaluation};
          }
        }
      }

      const fallbackCommand={
        steer:proposedSteer,
        accel:clamp(proposal.accel,minAccel,maxAccel),
      };
      const fallbackEval=best?null:evaluateCandidate(state,fallbackCommand,proposal,referenceAt,plantCfg);
      const selected=best??{...fallbackCommand,...fallbackEval};
      return {
        command:{steer:selected.steer,accel:selected.accel},
        diagnostics:{
          enabled:true,
          candidateCount,
          feasibleCount,
          fallbackUsed:!best,
          predictedFeasible:Boolean(best),
          cost:selected.cost,
          predictedMaxCrossTrack:selected.predictedMaxCte,
          predictedMaxHeadingError:selected.predictedMaxHeading,
          solveMs:performance.now()-start,
          horizon,
          predictionDt,
        },
      };
    },
  };
}
