import { createPIDController } from './pid.js';
import { wrapAngle } from '../models/kinematicBicycle.js';
const clamp=(v,lo,hi)=>Math.max(lo,Math.min(hi,v));
export function createUGVPathFollower(cfg={}){
  const dt=cfg.dt??0.02;
  const speedPid=createPIDController({
    kp:cfg.speedKp??1.8,ki:cfg.speedKi??0.35,kd:cfg.speedKd??0.08,
    uMin:cfg.minAccel??-3,uMax:cfg.maxAccel??2.5,antiWindup:0.45,
  },dt);
  let previousSteer=0;
  const maxSteer=cfg.maxSteer??0.55;
  const maxRate=cfg.maxSteerRate??0.9;
  const stanleyGain=cfg.stanleyGain??1.35;
  const softening=cfg.softening??0.8;
  return {
    update(state,ref){
      const headingError=wrapAngle(ref.heading-state.yaw);
      const crossTrackError=ref.y-state.y;
      const stanleyTerm=Math.atan2(stanleyGain*crossTrackError,Math.abs(state.v)+softening);
      const rawSteer=wrapAngle(headingError+stanleyTerm);
      const steerTarget=clamp(rawSteer,-maxSteer,maxSteer);
      const maxDelta=maxRate*dt;
      const steer=clamp(steerTarget,previousSteer-maxDelta,previousSteer+maxDelta);
      previousSteer=steer;
      const accel=speedPid.update(ref.speed,state.v);
      return {steer,accel,headingError,crossTrackError,rawSteer};
    },
    reset(){previousSteer=0;speedPid.reset();}
  };
}
