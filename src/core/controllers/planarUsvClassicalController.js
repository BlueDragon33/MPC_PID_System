import { createPIDController } from './pid.js';
import { wrapUsvAngle } from '../models/planarUsv.js';

const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));

export function createPlanarUsvClassicalController(cfg={}){
  const dt=cfg.dt??0.02;
  const minForce=cfg.minSurgeForce??-30;
  const maxForce=cfg.maxSurgeForce??45;
  const maxMoment=cfg.maxYawMoment??18;
  const maxForceRate=cfg.maxSurgeForceRate??90;
  const maxMomentRate=cfg.maxYawMomentRate??55;
  const speedPid=createPIDController({
    kp:cfg.speedKp??20,
    ki:cfg.speedKi??3.5,
    kd:cfg.speedKd??0.5,
    uMin:minForce,
    uMax:maxForce,
    antiWindup:0.35,
  },dt);
  const headingKp=cfg.headingKp??25;
  const headingKd=cfg.headingKd??11;
  const crossTrackGain=cfg.crossTrackGain??1.15;
  const softening=cfg.softening??0.8;
  let previous={surgeForce:0,yawMoment:0};

  return {
    update(state,reference){
      const crossTrackError=reference.y-state.y;
      const pathHeadingError=wrapUsvAngle(reference.heading-state.psi);
      const correction=Math.atan2(
        crossTrackGain*crossTrackError,
        Math.max(softening,Math.abs(state.u)+softening),
      );
      const desiredHeading=wrapUsvAngle(reference.heading+correction);
      const headingError=wrapUsvAngle(desiredHeading-state.psi);
      const speedError=reference.speed-state.u;
      const forceTarget=clamp(speedPid.update(reference.speed,state.u),minForce,maxForce);
      const momentTarget=clamp(
        headingKp*headingError-headingKd*state.r,
        -maxMoment,
        maxMoment,
      );
      const surgeForce=clamp(
        forceTarget,
        previous.surgeForce-maxForceRate*dt,
        previous.surgeForce+maxForceRate*dt,
      );
      const yawMoment=clamp(
        momentTarget,
        previous.yawMoment-maxMomentRate*dt,
        previous.yawMoment+maxMomentRate*dt,
      );
      previous={surgeForce,yawMoment};

      return {
        surgeForce,
        yawMoment,
        forceTarget,
        momentTarget,
        desiredHeading,
        crossTrackError,
        pathHeadingError,
        headingError,
        speedError,
      };
    },
    reset(){
      previous={surgeForce:0,yawMoment:0};
      speedPid.reset();
    },
  };
}
