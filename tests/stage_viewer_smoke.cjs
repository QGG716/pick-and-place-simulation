// Model-only DOM smoke test. This does NOT claim browser rendering or interaction.
const fs=require('fs'),path=require('path'),vm=require('vm');
const root=path.resolve(process.argv[2]);const elements=new Map();let content='';let plots=0;
const context=vm.createContext({console,devicePixelRatio:1.5,setInterval:()=>1,clearInterval:()=>{}});
class Element{
 constructor(id){this.id=id;this.style={};this.dataset={};this.children=[];this.options=[];this.clientWidth=900;this.clientHeight=410;this.value=0;this.classList={toggle(){}};}
 set innerHTML(v){this.html=v;if(this.id==='content')content=v;if(this.id==='nav')this.children=Array.from({length:10},(_,i)=>({dataset:{stage:i},classList:{toggle(){}}}));this.options=[...v.matchAll(/<option value="([^"]*)">([^<]*)<\/option>/g)].map(m=>({value:m[1],text:m[2]}));}
 get innerHTML(){return this.html||''}
 querySelectorAll(){return []}
 getContext(){plots++;return new Proxy({},{get:(_,key)=>()=>{}})}
 setPointerCapture(){}
}
const document={getElementById(id){if(['space','projection','fusionMode'].includes(id)&&!content.includes('id="'+id+'"'))return null;if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id)},createElement(tag){return new Element(tag)},body:{appendChild(s){vm.runInContext(fs.readFileSync(path.join(root,s.src),'utf8'),context)}}};
context.document=document;context.window=context;context.Image=class{set src(v){}};context.ResizeObserver=class{constructor(fn){this.fn=fn}observe(){this.fn()}disconnect(){}};
vm.runInContext(fs.readFileSync(path.join(root,'data.js'),'utf8'),context);
let html=fs.readFileSync(path.join(root,'index.html'),'utf8');vm.runInContext(html.split('<script src="data.js"></script><script>')[1].split('</script>')[0],context);
let cases=0,empty=0;
for(let gi=0;gi<context.STAGE_DATA.groups.length;gi++){
 context.stageViewer.selectGroup(gi);const g=context.STAGE_DATA.groups[gi];
 for(let mi=0;mi<g.modules.length;mi++)for(let ii=0;ii<g.modules[mi].instances.length;ii++){
  vm.runInContext(`mi=${mi};ii=${ii};matchObject();changed()`,context);
  for(let s of [0,1,2,3,4,5]){context.stageViewer.go(s);cases++;}
 }
 for(let oi=0;oi<g.objects.length;oi++){
  vm.runInContext(`oi=${oi};ci=0;changed()`,context);
  for(let mode of ['pre','post','object']){vm.runInContext(`fusionMode='${mode}'`,context);context.stageViewer.go(6);cases++;}
  context.stageViewer.go(8);cases++;
  if(!g.objects[oi].surfaces.length)empty++;
 }
 for(let width of [600,1200]){elements.get('space').clientWidth=width;vm.runInContext('draw3D?.()',context);}
}
console.log(JSON.stringify({status:'PASS',kind:'SIMULATED_DOM_NOT_BROWSER',cases,empty_objects:empty,plots,projection_image_loading:'NOT_TESTED',browser_interaction:'NOT_AVAILABLE'}));
