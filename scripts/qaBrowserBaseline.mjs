import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { chromium } from 'playwright';

const baseUrl=process.env.QA_BASE_URL||'http://127.0.0.1:4173/';
const outDir=path.resolve('artifacts/qa-e1');
fs.mkdirSync(outDir,{recursive:true});

const browser=await chromium.launch({headless:true});
const page=await browser.newPage({viewport:{width:1440,height:900}});
const consoleErrors=[];
const pageErrors=[];
const report={
  baseUrl,
  invalidImportRejected:false,
  validImportAccepted:false,
  storageFailureRecovered:false,
  mobile:null,
  consoleErrors,
  pageErrors,
  error:null,
};
page.on('console',(message)=>{ if(message.type()==='error') consoleErrors.push(message.text()); });
page.on('pageerror',(error)=>pageErrors.push(error.message));

await page.goto(baseUrl,{waitUntil:'networkidle'});
assert.equal(await page.locator('h1').count(),1,'workbench must expose exactly one primary heading');
assert.match(await page.locator('h1').innerText(),/Linear control benchmark/);
assert(await page.locator('.research-context-header').isVisible(),'research context must be visible');
assert(await page.locator('.context-safety').isVisible(),'actual safety context must be visible');
assert(await page.getByRole('button',{name:/Run Simulation/i}).isVisible(),'primary Run action must be visible');
assert.equal(await page.locator('svg.timeseries-chart title').count()>0,true,'control chart must expose a title');
assert.equal(await page.locator('svg.timeseries-chart desc').count()>0,true,'control chart must expose a textual description');

for(const name of ['Analysis','Scenarios','Documentation','Settings','Simulation']){
  await page.getByRole('button',{name,exact:true}).click();
  await page.waitForTimeout(60);
}
assert.match(page.url(),/#simulation$/);

const invalidPayload=JSON.stringify({
  schema:'mpc-pid-experiment/v1',
  name:'invalid-zero-dt',
  presetId:'custom',
  config:{dt:0,duration:2,mpc:{}},
});
await page.locator('.hidden-file-input').setInputFiles({
  name:'invalid-zero-dt.json',
  mimeType:'application/json',
  buffer:Buffer.from(invalidPayload),
});
await page.locator('.experiment-status').waitFor();
assert.match(await page.locator('.experiment-status').innerText(),/Import failed:.*dt must be > 0/i);
report.invalidImportRejected=true;

const validPayload=JSON.stringify({
  schema:'mpc-pid-experiment/v1',
  name:'qa-valid',
  presetId:'custom',
  config:{dt:0.02,duration:1,mpc:{}},
});
await page.locator('.hidden-file-input').setInputFiles({
  name:'qa-valid.json',
  mimeType:'application/json',
  buffer:Buffer.from(validPayload),
});
assert.match(await page.locator('.experiment-status').innerText(),/Imported qa-valid\.json/);
report.validImportAccepted=true;
await page.getByRole('button',{name:/Run Simulation/i}).click();

await page.evaluate(()=>{
  Storage.prototype.setItem=function(){ throw new DOMException('quota exceeded','QuotaExceededError'); };
});
await page.getByRole('button',{name:'Save',exact:true}).click();
assert.match(await page.locator('.experiment-status').innerText(),/Local storage is unavailable/i);
report.storageFailureRecovered=true;

await page.setViewportSize({width:390,height:844});
await page.waitForTimeout(100);
const mobile=await page.evaluate(()=>({
  viewport:window.innerWidth,
  scrollWidth:document.documentElement.scrollWidth,
  statusVisible:!!document.querySelector('.app-status')&&getComputedStyle(document.querySelector('.app-status')).display!=='none',
  safetyVisible:!!document.querySelector('.context-safety')&&document.querySelector('.context-safety').getBoundingClientRect().height>0,
}));
assert.equal(mobile.scrollWidth,mobile.viewport,'mobile baseline must not horizontally overflow');
assert.equal(mobile.statusVisible,true,'mobile application status must remain visible');
assert.equal(mobile.safetyVisible,true,'mobile actual-safety context must remain visible');
report.mobile=mobile;

await page.screenshot({path:path.join(outDir,'qa-e1-mobile-390.png'),fullPage:true});
await page.setViewportSize({width:1440,height:900});
await page.screenshot({path:path.join(outDir,'qa-e1-desktop-1440.png'),fullPage:true});

assert.deepEqual(pageErrors,[],'browser must not raise uncaught page errors');
assert.deepEqual(consoleErrors,[],'browser must not emit console errors');

await browser.close();
fs.writeFileSync(path.join(outDir,'browser-report.json'),JSON.stringify(report,null,2)+'\n');

console.log('QA-E1 browser acceptance: PASS');
