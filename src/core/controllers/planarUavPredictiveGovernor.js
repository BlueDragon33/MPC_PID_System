import { stepPlanarUav } from '../models/planarUav.js';

const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

function normalizeOffsets(values,fallback){
  const source=Array.isArray(values)&&values.length?values:fallback;
  return [...new Set(source.map(v=>Number(v.toFixed(12))))].sort((a,b)=>a-b);
}

export function createPlanarUavPredictiveGovernor(cfg={}){
  const predictionDt=cfg.predictionDt??0.05;
  const horizon=cfg.horizon??8;
  const minThrust=cfg.minThrust??0;
  const maxThrust=cfg.maxThrust??30.22;
  const maxTorque=cfg.maxTorque??0.7;
  const mass=cfg.mass??1.4;
  const gravity=cfg.gravity??9.81;
  const hoverThrust=mass*gravity;
  const maxAbsXError=cfg.maxAbsXError??0.9;
  const maxAbsZError=cfg.maxAbsZError??0.8;
  const maxAbsTilt=cfg.maxAbsTilt??0.55;
  const maxAbsVx=cfg.maxAbsVx??3.5;
  const maxAbsVz=cfg.maxAbsVz??2.5;
  const minAltitude=cfg.minAltitude??0.5;
  const maxAltitude=cfg.maxAltitude??3.2;
  const thrustOffsets=normalizeOffsets(cfg.thrustOffsets,[-0.18,0,0.18]);
  const torqueOffsets=normalizeOffsets(cfg.torqueOffsets,[-0.08,-0.04,0,0.04,0.08]);
  const weights={
    x:cfg.qX??5,
    z:cfg.qZ??20,
    vx:cfg.qVx??2,
    vz:cfg.qVz??8,
    theta:cfg.qTheta??2,
    q:cfg.qQ??0.15,
    thrust:cfg.rThrust??0.5,
    torque:cfg.rTorque??0.06,
    proposal:cfg.rProposal??0.12,
  };

  function boundedCommand(command){
    return {
      thrust:clamp(command.thrust,minThrust,maxThrust),
      torque:clamp(command.torque,-maxTorque,maxTorque),
    };
  }

  function evaluateCandidate({state,time,proposal,offset,referenceAt,baselineAt,plantCfg}){
    let s={...state};
    let cost=0;
    let feasible=true;
    let predictedMaxXError=0;
    let predictedMaxZError=0;
    let predictedMaxTilt=0;
    let predictedMaxVx=0;
    let predictedMaxVz=0;

    for(let i=0;i<horizon;i+=1){
      const commandTime=time+i*predictionDt;
      const commandRef=referenceAt(commandTime);
      const baseline=i===0?proposal:baselineAt(s,commandRef);
      const command=boundedCommand({
        thrust:baseline.thrust+offset.thrust,
        torque:baseline.torque+offset.torque,
      });
      s=stepPlanarUav(s,command,{...plantCfg,dt:predictionDt});
      const ref=referenceAt(time+(i+1)*predictionDt);
      const xError=ref.x-s.x;
      const zError=ref.z-s.z;
      const vxError=ref.vx-s.vx;
      const vzError=ref.vz-s.vz;

      predictedMaxXError=Math.max(predictedMaxXError,Math.abs(xError));
      predictedMaxZError=Math.max(predictedMaxZError,Math.abs(zError));
      predictedMaxTilt=Math.max(predictedMaxTilt,Math.abs(s.theta));
      predictedMaxVx=Math.max(predictedMaxVx,Math.abs(s.vx));
      predictedMaxVz=Math.max(predictedMaxVz,Math.abs(s.vz));

      if(
        Math.abs(xError)>maxAbsXError ||
        Math.abs(zError)>maxAbsZError ||
        Math.abs(s.theta)>maxAbsTilt ||
        Math.abs(s.vx)>maxAbsVx ||
        Math.abs(s.vz)>maxAbsVz ||
        s.z<minAltitude ||
        s.z>maxAltitude
      ){
        feasible=false;
        break;
      }

      const normalizedThrust=command.thrust/hoverThrust-1;
      const normalizedTorque=command.torque/maxTorque;
      cost+=
        weights.x*xError*xError+
        weights.z*zError*zError+
        weights.vx*vxError*vxError+
        weights.vz*vzError*vzError+
        weights.theta*s.theta*s.theta+
        weights.q*s.q*s.q+
        weights.thrust*normalizedThrust*normalizedThrust+
        weights.torque*normalizedTorque*normalizedTorque;
    }

    cost+=weights.proposal*(
      (offset.thrust/hoverThrust)*(offset.thrust/hoverThrust)+
      (offset.torque/maxTorque)*(offset.torque/maxTorque)
    );

    return {
      feasible,
      cost,
      predictedMaxXError,
      predictedMaxZError,
      predictedMaxTilt,
      predictedMaxVx,
      predictedMaxVz,
    };
  }

  return {
    select({state,time,proposal,referenceAt,baselineAt,plantCfg}){
      const start=performance.now();
      let best=null;
      let candidateCount=0;
      let feasibleCount=0;

      for(const thrustOffset of thrustOffsets){
        for(const torqueOffset of torqueOffsets){
          candidateCount+=1;
          const offset={thrust:thrustOffset,torque:torqueOffset};
          const evaluation=evaluateCandidate({
            state,time,proposal,offset,referenceAt,baselineAt,plantCfg,
          });
          if(!evaluation.feasible) continue;
          feasibleCount+=1;
          if(!best || evaluation.cost<best.cost){
            best={offset,...evaluation};
          }
        }
      }

      const fallbackOffset={thrust:0,torque:0};
      const fallbackEvaluation=best?null:evaluateCandidate({
        state,time,proposal,offset:fallbackOffset,referenceAt,baselineAt,plantCfg,
      });
      const selected=best??{offset:fallbackOffset,...fallbackEvaluation};
      const command=boundedCommand({
        thrust:proposal.thrust+selected.offset.thrust,
        torque:proposal.torque+selected.offset.torque,
      });

      return {
        command,
        diagnostics:{
          enabled:true,
          candidateCount,
          feasibleCount,
          fallbackUsed:!best,
          predictedFeasible:Boolean(best),
          cost:selected.cost,
          thrustOffset:selected.offset.thrust,
          torqueOffset:selected.offset.torque,
          predictedMaxXError:selected.predictedMaxXError,
          predictedMaxZError:selected.predictedMaxZError,
          predictedMaxTilt:selected.predictedMaxTilt,
          predictedMaxVx:selected.predictedMaxVx,
          predictedMaxVz:selected.predictedMaxVz,
          solveMs:performance.now()-start,
          horizon,
          predictionDt,
        },
      };
    },
  };
}
