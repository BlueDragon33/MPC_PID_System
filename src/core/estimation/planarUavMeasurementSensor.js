import { createSeededGaussian } from './deterministicNoise.js';
import { wrapPlanarAngle } from '../models/planarUav.js';

export function createPlanarUavMeasurementSensor({
  seed=20260928,
  noiseStd={x:0.08,z:0.06,theta:0.015,vx:0.08,vz:0.07,q:0.04},
  bias={x:0,z:0,theta:0,vx:0,vz:0,q:0},
}={}){
  const gaussian=createSeededGaussian(seed);
  return {
    read(state){
      const noise={
        x:gaussian(0,noiseStd.x??0),
        z:gaussian(0,noiseStd.z??0),
        theta:gaussian(0,noiseStd.theta??0),
        vx:gaussian(0,noiseStd.vx??0),
        vz:gaussian(0,noiseStd.vz??0),
        q:gaussian(0,noiseStd.q??0),
      };
      return {
        x:state.x+(bias.x??0)+noise.x,
        z:state.z+(bias.z??0)+noise.z,
        theta:wrapPlanarAngle(state.theta+(bias.theta??0)+noise.theta),
        vx:state.vx+(bias.vx??0)+noise.vx,
        vz:state.vz+(bias.vz??0)+noise.vz,
        q:state.q+(bias.q??0)+noise.q,
        noise,
      };
    },
  };
}
