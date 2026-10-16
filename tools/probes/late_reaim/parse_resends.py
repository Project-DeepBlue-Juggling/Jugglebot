"""Parse every RESEND / RESEND-REFUSED line of the 2026-10-16 launch logs (late-catch re-aim bump design,
plans/active/late-catch-reaim-bump.md): pattern, skill, code, delta mm / s, time to the committed landing,
and the limit the preceding `install_segment refused` line names.  Usage: python parse_resends.py <out.json>
"""
import re,sys,json
LOG_ROOT = '/home/jetson/.ros/log'
logs={'am':'2026-10-16-10-50-42-798495-jetson-1033523','pm1':'2026-10-16-16-10-37-689586-jetson-1220502','pm2':'2026-10-16-16-23-01-479044-jetson-1230939'}
rows=[]
for tag,d in logs.items():
    pat=None; aims={}; last_ref=None
    for ln in open('/home/jetson/.ros/log/%s/launch.log'%d,errors='replace'):
        m=re.search(r"pattern='([a-z_0-9]+)'",ln)
        if m: pat=m.group(1); aims={}
        m=re.search(r"(\d+\.\d+) CATCH-AIM skill (\d+): source=(\S+) landing=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\) mm t_land=(\d+\.\d+)",ln)
        if m: aims[int(m.group(2))]=dict(t=float(m.group(1)),src=m.group(3),xy=(float(m.group(4)),float(m.group(5))),tl=float(m.group(7)))
        if 'install_segment refused' in ln: last_ref=ln
        m=re.search(r"(\d+\.\d+) RESEND-REFUSED (\S+) at skill (\d+): the fit moved the landing ([\d.]+) mm / ([-+\d.]+) s",ln)
        m2=re.search(r"(\d+\.\d+) RESEND skill (\d+): the fit moved the landing ([\d.]+) mm / ([-+\d.]+) s",ln)
        if m or m2:
            if m: t,code,k,mm,ds=float(m.group(1)),m.group(2),int(m.group(3)),float(m.group(4)),float(m.group(5))
            else: t,k,mm,ds=float(m2.group(1)),int(m2.group(2)),float(m2.group(3)),float(m2.group(4)); code='INSTALLED'
            a=aims.get(k,{}); r={'log':tag,'pat':pat,'t':t,'skill':k,'code':code,'mm':mm,'ds':ds,'tl':a.get('tl'),'src':a.get('src')}
            r['to_land']= (a['tl']-t) if a else None
            if m and last_ref:
                for key,rx in [('ks',r'k_s (\d+)'),('kw',r'k_wire@node (\d+)'),('tviol',r't=([\d.]+) s'),('peak',r'peak [^>]*?([\d.]+) [a-z/³^0-9]* >'),('limit',r'\((\w+):')]:
                    mm_=re.search(rx,last_ref); r[key]=mm_.group(1) if mm_ else None
                mw=re.search(r'\(([A-Z_0-9]+: [^\[]{0,140})',last_ref); r['why']=mw.group(1) if mw else None
            if m: last_ref=None
            rows.append(r)
json.dump(rows,open(sys.argv[1],'w'),indent=0)
import collections
for r in rows:
    print(r['log'],r['pat'],r['skill'],r['code'],r['mm'],r['ds'],'toland=%.3f'%r['to_land'] if r['to_land'] else None,r.get('ks'),r.get('kw'),r.get('tviol'),(r.get('why') or '')[:90])
