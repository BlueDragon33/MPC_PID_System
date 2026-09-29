import { createSeededGaussian } from './deterministicNoise.js';
import { wrapUsvAngle } from '../models/planarUsv.js';

export function createPlanarUsvMeasurementSensor({
  seed=20260929,
  noiseStd={x:0.12,y:0.12,psi:0.02,u:0.08,v:0.05,r:0.025},
  bias={x:0,y:0,psi:0,u:0,v:0,r:0},
}={}){
  const gaussian=createSeededGaussian(seed);
  return {
    read(state){
      const noise={
        x:gaussian(0,noiseStd.x??0),
        y:gaussian(0,noiseStd.y??0),
        psi:gaussian(0,noiseStd.psi??0),
        u:gaussian(0,noiseStd.u??0),
        v:gaussian(0,noiseStd.v??0),
        r:gaussian(0,noiseStd.r??0),
      };
      return {
        x:state.x+(bias.x??0)+noise.x,
        y:state.y+(bias.y??0)+noise.y,
        psi:wrapUsvAngle(state.psi+(bias.psi??0)+noise.psi),
        u:state.u+(bias.u??0)+noise.u,
        v:state.v+(bias.v??0)+noise.v,
        r:state.r+(bias.r??0)+noise.r,
        noise,
      };
    },
  };
}
