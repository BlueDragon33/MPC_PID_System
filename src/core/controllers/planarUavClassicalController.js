import { wrapPlanarAngle } from '../models/planarUav.js';

const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

export function createPlanarUavClassicalController(cfg={}){
  const mass=cfg.mass??1.4;
  const gravity=cfg.gravity??9.81;
  const maxTilt=cfg.maxTilt??0.45;
  const maxTorque=cfg.maxTorque??0.7;
  const minThrust=cfg.minThrust??0;
  const maxThrust=cfg.maxThrust??2.2*mass*gravity;

  const gains={
    xP:cfg.xP??1.2,
    xD:cfg.xD??1.8,
    zP:cfg.zP??2.4,
    zD:cfg.zD??2.0,
    thetaP:cfg.thetaP??9.0,
    thetaD:cfg.thetaD??2.2,
  };

  return {
    update(state,reference){
      const xError=reference.x-state.x;
      const vxError=reference.vx-state.vx;
      const zError=reference.z-state.z;
      const vzError=reference.vz-state.vz;

      const axCommand=gains.xP*xError+gains.xD*vxError;
      const azCommand=gains.zP*zError+gains.zD*vzError;
      const verticalSpecificForce=Math.max(0.1,gravity+azCommand);
      const desiredTheta=clamp(
        Math.atan2(-axCommand,verticalSpecificForce),
        -maxTilt,
        maxTilt,
      );
      const thrust=clamp(
        mass*Math.hypot(axCommand,verticalSpecificForce),
        minThrust,
        maxThrust,
      );
      const attitudeError=wrapPlanarAngle(desiredTheta-state.theta);
      const torque=clamp(
        gains.thetaP*attitudeError-gains.thetaD*state.q,
        -maxTorque,
        maxTorque,
      );

      return {
        thrust,
        torque,
        desiredTheta,
        xError,
        zError,
        vxError,
        vzError,
        attitudeError,
        axCommand,
        azCommand,
      };
    },
  };
}
