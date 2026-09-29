import fs from 'node:fs';
import path from 'node:path';

const root=process.cwd();

function walk(dir){
  if(!fs.existsSync(dir)) return [];
  const out=[];
  for(const entry of fs.readdirSync(dir,{withFileTypes:true})){
    const full=path.join(dir,entry.name);
    if(entry.isDirectory()) out.push(...walk(full));
    else out.push(full);
  }
  return out;
}

const rel=file=>path.relative(root,file).split(path.sep).join('/');
const sourceFiles=walk(path.join(root,'src')).filter(file=>/\.(js|jsx|mjs)$/.test(file));
const importRegex=/(?:import\s+(?:[^'"]+?\s+from\s+)?|export\s+[^'"]*?\s+from\s+)['"]([^'"]+)['"]/g;

function importsFor(file){
  const text=fs.readFileSync(file,'utf8');
  return [...text.matchAll(importRegex)].map(match=>match[1]);
}

const orchestrationFiles=sourceFiles.filter(file=>rel(file).startsWith('src/core/orchestration/'));
const vehicleOrchestrators=orchestrationFiles
  .map(rel)
  .filter(file=>/(ugv|uav|usv)/i.test(file));

const vehicleProviderCoupling=[];
for(const file of orchestrationFiles){
  for(const specifier of importsFor(file)){
    if(!specifier.startsWith('.')) continue;
    if(/\.\.\/(models|controllers|estimation|mpc|solvers|safety)\//.test(specifier)){
      vehicleProviderCoupling.push({source:rel(file),target:specifier});
    }
  }
}

const applicationFiles=sourceFiles.filter(file=>rel(file).startsWith('src/application/'));
const applicationCoreImports=[];
for(const file of applicationFiles){
  for(const specifier of importsFor(file)){
    if(specifier.includes('/core/')) applicationCoreImports.push({source:rel(file),target:specifier});
  }
}

const presentationFiles=sourceFiles.filter(file=>
  rel(file)==='src/App.jsx' ||
  rel(file)==='src/main.jsx' ||
  rel(file).startsWith('src/components/')
);
const presentationCoreImports=[];
for(const file of presentationFiles){
  for(const specifier of importsFor(file)){
    if(specifier.includes('/core/')) presentationCoreImports.push({source:rel(file),target:specifier});
  }
}

const registryFiles=sourceFiles.map(rel).filter(file=>/registry/i.test(file));
const providerFiles=sourceFiles.map(rel).filter(file=>/provider/i.test(file));
const codeManifestFiles=walk(path.join(root,'src')).map(rel).filter(file=>/manifest\.(json|js|mjs)$/i.test(file));

const specificRuntimeFiles=[
  'src/core/orchestration/ugvBicycleSimulator.js',
  'src/core/orchestration/planarUavSimulator.js',
  'src/core/orchestration/planarUsvSimulator.js',
];
const missingSpecificRuntimeFiles=specificRuntimeFiles.filter(file=>!fs.existsSync(path.join(root,file)));

const report={
  schema:'mpc-pid-extensibility-audit/v1',
  classification:'ARCHITECTURE_BASELINE_EVIDENCE',
  sourceOfTruth:'.blueprint/work-packages.json',
  invariants:{
    presentationDirectCoreImports:presentationCoreImports.length,
    missingVehicleRuntimeFiles:missingSpecificRuntimeFiles,
  },
  coupling:{
    vehicleSpecificOrchestrators:vehicleOrchestrators,
    vehicleSpecificOrchestratorCount:vehicleOrchestrators.length,
    orchestrationDirectProviderImports:vehicleProviderCoupling,
    orchestrationDirectProviderImportCount:vehicleProviderCoupling.length,
    applicationDirectCoreImports:applicationCoreImports,
    applicationDirectCoreImportCount:applicationCoreImports.length,
  },
  extensionSurface:{
    registryFiles,
    registryFileCount:registryFiles.length,
    providerNamedFiles:providerFiles,
    providerNamedFileCount:providerFiles.length,
    codeManifestFiles,
    codeManifestFileCount:codeManifestFiles.length,
  },
  interpretation:{
    registryFoundationPresent:registryFiles.length>0,
    providerPackagingPresent:providerFiles.length>0 || codeManifestFiles.length>0,
    universalVehicleRuntimePresent:vehicleOrchestrators.length<=1,
  },
};

if(presentationCoreImports.length){
  console.error('E1 audit FAIL: presentation imports core directly',presentationCoreImports);
  process.exit(1);
}
if(missingSpecificRuntimeFiles.length){
  console.error('E1 audit FAIL: expected current-main vehicle runtimes are missing',missingSpecificRuntimeFiles);
  process.exit(1);
}
if(vehicleOrchestrators.length<3){
  console.error('E1 audit FAIL: current architecture baseline changed unexpectedly; re-audit before migration.');
  process.exit(1);
}

console.log('E1 current extensibility audit: PASS');
console.log(JSON.stringify(report,null,2));
