import fs from 'node:fs';
import path from 'node:path';
import { chromium } from 'playwright';

const baseUrl=process.env.UI_AUDIT_URL||'http://127.0.0.1:4173/';
const outDir=path.resolve('artifacts/ui-audit');
fs.mkdirSync(outDir,{recursive:true});

const viewports=[
  {name:'desktop-1920',width:1920,height:1080},
  {name:'desktop-1440',width:1440,height:1000},
  {name:'desktop-1280',width:1280,height:900},
  {name:'tablet-1024',width:1024,height:900},
  {name:'tablet-768',width:768,height:1024},
  {name:'mobile-430',width:430,height:932},
  {name:'mobile-390',width:390,height:844},
];

const browser=await chromium.launch({headless:true});
const report={
  schema:'mpc-pid-ui-visual-audit/v1',
  baseUrl,
  viewports:[],
  defects:[],
};

for(const viewport of viewports){
  const page=await browser.newPage({viewport:{width:viewport.width,height:viewport.height}});
  await page.goto(baseUrl,{waitUntil:'networkidle'});
  await page.screenshot({
    path:path.join(outDir,`${viewport.name}-dark.png`),
    fullPage:true,
  });

  const snapshot=await page.evaluate(()=>{
    const visible=el=>{
      const s=getComputedStyle(el);
      const r=el.getBoundingClientRect();
      return s.display!=='none'&&s.visibility!=='hidden'&&r.width>0&&r.height>0;
    };
    const tiny=[...document.querySelectorAll('body *')].filter(el=>{
      if(!visible(el)||el.children.length>0||!el.textContent.trim()) return false;
      const size=parseFloat(getComputedStyle(el).fontSize);
      return Number.isFinite(size)&&size<11;
    }).slice(0,40).map(el=>({
      tag:el.tagName.toLowerCase(),
      text:el.textContent.trim().slice(0,70),
      fontSize:getComputedStyle(el).fontSize,
      className:el.className,
    }));
    const top=document.querySelector('.app-topbar');
    const status=document.querySelector('.app-status');
    const sidebar=document.querySelector('.control-sidebar');
    const main=document.querySelector('.dashboard-main,[role="main"],main');
    return {
      title:document.title,
      scrollWidth:document.documentElement.scrollWidth,
      viewportWidth:window.innerWidth,
      horizontalOverflow:document.documentElement.scrollWidth>window.innerWidth+1,
      bodyHeight:document.body.scrollHeight,
      topbarVisible:Boolean(top&&visible(top)),
      statusVisible:Boolean(status&&visible(status)),
      sidebarVisible:Boolean(sidebar&&visible(sidebar)),
      mainVisible:Boolean(main&&visible(main)),
      tinyTextCount:tiny.length,
      tinyText:tiny,
      hasResearchContextHeader:Boolean(document.querySelector('.research-context-header')),
      hasThemeControl:Boolean(document.querySelector('[data-theme-control],.theme-control')),
      safetyContextVisible:Boolean(document.querySelector('.context-safety')&&visible(document.querySelector('.context-safety'))),
      theme:document.documentElement.dataset.theme||'unset',
      h1Count:document.querySelectorAll('h1').length,
      navButtonCount:document.querySelectorAll('.top-nav button').length,
      focusableCount:document.querySelectorAll('button,a[href],input,select,textarea,[tabindex]:not([tabindex="-1"])').length,
    };
  });

  if(snapshot.horizontalOverflow) report.defects.push({viewport:viewport.name,id:'horizontal-overflow-dark'});
  if(!snapshot.statusVisible&&viewport.width<=430) report.defects.push({viewport:viewport.name,id:'mobile-status-hidden'});
  if(snapshot.tinyTextCount>0) report.defects.push({viewport:viewport.name,id:'tiny-text',count:snapshot.tinyTextCount});
  if(!snapshot.hasResearchContextHeader) report.defects.push({viewport:viewport.name,id:'missing-research-context-header'});
  if(!snapshot.hasThemeControl) report.defects.push({viewport:viewport.name,id:'missing-theme-control'});
  if(!snapshot.safetyContextVisible) report.defects.push({viewport:viewport.name,id:'missing-safety-context'});

  let light=null;
  if(snapshot.hasThemeControl){
    await page.locator('[data-theme-control]').click();
    await page.waitForTimeout(120);
    light=await page.evaluate(()=>({
      theme:document.documentElement.dataset.theme||'unset',
      scrollWidth:document.documentElement.scrollWidth,
      viewportWidth:window.innerWidth,
      horizontalOverflow:document.documentElement.scrollWidth>window.innerWidth+1,
    }));
    await page.screenshot({
      path:path.join(outDir,`${viewport.name}-light.png`),
      fullPage:true,
    });
    if(light.theme!=='light') report.defects.push({viewport:viewport.name,id:'light-theme-not-applied'});
    if(light.horizontalOverflow) report.defects.push({viewport:viewport.name,id:'horizontal-overflow-light'});
  }

  report.viewports.push({...viewport,...snapshot,light});
  await page.close();
}

const page=await browser.newPage({viewport:{width:1440,height:1000}});
await page.goto(baseUrl,{waitUntil:'networkidle'});
await page.keyboard.press('Tab');
const keyboard=await page.evaluate(()=>({
  activeTag:document.activeElement?.tagName??null,
  activeClass:document.activeElement?.className??null,
  hasVisibleFocus:document.activeElement
    ?['outlineStyle','outlineWidth'].reduce((acc,key)=>{
      const s=getComputedStyle(document.activeElement);
      acc[key]=s[key];
      return acc;
    },{})
    :null,
}));
report.keyboard=keyboard;
await page.close();

await browser.close();
fs.writeFileSync(path.join(outDir,'report.json'),JSON.stringify(report,null,2)+'\n');
console.log('UI visual audit complete');
console.log(JSON.stringify(report,null,2));
if(report.defects.length){
  console.error(`UI visual audit FAIL: ${report.defects.length} defect(s)`);
  process.exit(1);
}
console.log('UI visual audit PASS');
