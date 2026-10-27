"""Solve starvation scan: the planner's solve wall time against its own CPU time
and the box load, per Sitting (JB-76).

Reads a launch.log whose ``solve X ms (cpu Y ms, ivcsw N, vcsw M)`` suffix the
solver worker has carried since 2026-10-28, and a ``profile_session.py``
directory (pidstat.log, vmstat.log) recorded over the same Sitting. Prints,
per solve kind, wall vs CPU vs involuntary context switches; the box's total
%CPU per second and the processes behind it; and the CATCH wall/CPU ratio
binned by the box load in the same second. A CPU time near the wall time is a
slow solve; a CPU time well under it with many involuntary switches is a
starved one.

Motivating entry: logbook/2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md.

Usage (repo root, project venv)::

    python tools/probes/solve_starvation_scan.py <launch.log> <profile_dir> [--tz-hours 11]

Output goes to stdout; tee it to temp/probes/.
"""
import argparse,re,collections,statistics as st,datetime as dt
ap=argparse.ArgumentParser(description=__doc__.split('\n')[0])
ap.add_argument('launch_log'); ap.add_argument('profile_dir')
ap.add_argument('--tz-hours',type=float,default=11.0,help='local offset of the pidstat/vmstat clocks (AEDT = 11)')
args=ap.parse_args()
D=args.profile_dir; L=args.launch_log
TZ=dt.timezone(dt.timedelta(hours=args.tz_hours))
def hms(epoch): return dt.datetime.fromtimestamp(epoch,TZ).strftime('%H:%M:%S')
# --- solves
sol=[]
rx=re.compile(r'^(\d+\.\d+) .*?(install_segment (\w+) .*?|check=([\w:-]+), )solve ([\d.]+) ms \(cpu ([\d.]+) ms, ivcsw (\d+), vcsw (\d+)\)')
for ln in open(L,errors='replace'):
    m=rx.search(ln)
    if m:
        # A refusal line carries no kind; in practice it is a CATCH (a re-aim
        # RESEND or a feed catch), but it is reported on its own row.
        kind=m.group(3) or 'REFUSED'
        sol.append(dict(t=float(m.group(1)),kind=kind,ref=m.group(4) or 'ok',wall=float(m.group(5)),cpu=float(m.group(6)),iv=int(m.group(7))))
print('solves parsed',len(sol))
def pct(x,p): x=sorted(x); return x[min(len(x)-1,int(p*len(x)))]
for k in ('CATCH','THROW','REST','REFUSED'):
    r=[s for s in sol if s['kind']==k]
    if not r: continue
    w=[s['wall'] for s in r]; c=[s['cpu'] for s in r]; iv=[s['iv'] for s in r]; ratio=[s['wall']/s['cpu'] for s in r]
    print('%-5s n=%3d wall p50 %5.1f p90 %5.1f max %5.1f | cpu p50 %5.1f p90 %5.1f max %5.1f | wall/cpu p50 %.2f p90 %.2f | ivcsw p50 %d p90 %d max %d'%(k,len(r),pct(w,.5),pct(w,.9),max(w),pct(c,.5),pct(c,.9),max(c),pct(ratio,.5),pct(ratio,.9),pct(iv,.5),pct(iv,.9),max(iv)))
cat=[s for s in sol if s['kind'] in ('CATCH','REFUSED')]
print('refusal checks:',dict(collections.Counter(s['ref'] for s in sol if s['kind']=='REFUSED')))
print('CATCH+REFUSED (refusals carry no kind; assumed CATCH) wall>150:',sum(s['wall']>150 for s in cat),'of',len(cat),'; cpu>150:',sum(s['cpu']>150 for s in cat),'; cpu>100:',sum(s['cpu']>100 for s in cat))
# ivcsw per cpu-ms
if cat: print('ivcsw per 10 ms cpu, CATCH+REFUSED: p50 %.1f p90 %.1f'%(pct([10*s['iv']/s['cpu'] for s in cat],.5),pct([10*s['iv']/s['cpu'] for s in cat],.9)))
# --- pidstat by second
def tosec(h): a,b,c=h.split(':'); return int(a)*3600+int(b)*60+int(c)
rows=collections.defaultdict(dict)
for ln in open(D+'/pidstat.log'):
    p=ln.split()
    if len(p)<21 or ':' not in p[0]: continue
    try: rows[tosec(p[0])][int(p[2])]=(float(p[7]),float(p[19]),' '.join(p[20:]))
    except ValueError: pass
secs=sorted(rows)
tot={t:sum(v[0] for v in rows[t].values()) for t in secs}
agg=collections.Counter(); nv=collections.Counter(); n=collections.Counter()
for t in secs:
    for pid,(cpu,ncs,cmd) in rows[t].items(): agg[(pid,cmd)]+=cpu; nv[(pid,cmd)]+=ncs; n[(pid,cmd)]+=1
print('\npidstat seconds',len(secs),'total %%CPU per second: mean %.0f p50 %.0f p90 %.0f max %.0f (600 = 6 cores)'%(st.mean(tot.values()),pct(list(tot.values()),.5),pct(list(tot.values()),.9),max(tot.values())))
print('%-8s %-28s %6s %6s %5s %8s'%('pid','cmd','cpu-s','mean%','secs','nvcsw/s'))
for k,v in agg.most_common(16): print('%-8d %-28s %6.0f %6.0f %5d %8.0f'%(k[0],k[1][:28],v/100,v/n[k],n[k],nv[k]/n[k]))
# --- vmstat r per second
vm={}
for ln in open(D+'/vmstat.log'):
    p=ln.split()
    if len(p)>=19 and p[0].isdigit(): vm[tosec(p[18])]=int(p[0])
# --- correlate: for each CATCH, the box total %CPU and r in that second
print('\nCATCH+REFUSED wall/cpu vs box load in the same second')
bins=collections.defaultdict(list)
for s in cat:
    sec=tosec(hms(s['t']))
    T=tot.get(sec); r=vm.get(sec)
    if T is None: continue
    b=int(T//100)*100
    bins[b].append((s['wall'],s['cpu'],s['iv'],r or 0))
for b in sorted(bins):
    v=bins[b]
    print('box %3d-%3d%%: n=%3d wall p50 %5.1f cpu p50 %5.1f ivcsw p50 %4d runq p50 %4.1f'%(b,b+100,len(v),pct([x[0] for x in v],.5),pct([x[1] for x in v],.5),pct([x[2] for x in v],.5),pct([x[3] for x in v],.5)))
# idle vs juggling box load
goal_secs=set()
rg=re.compile(r'^(\d+\.\d+) .*(goal dispatched|Juggle (ended|OK))')
ev=[(float(m.group(1)),'start' if 'dispatched' in m.group(2) else 'end') for ln in open(L,errors='replace') for m in [rg.search(ln)] if m]
cur=None
for t,e in ev:
    if e=='start': cur=t
    elif cur: goal_secs.update(range(tosec(hms(cur)),tosec(hms(t))+1)); cur=None
inj=[tot[t] for t in secs if t in goal_secs] or [0]; outj=[tot[t] for t in secs if t not in goal_secs and t>secs[0]+20] or [0]
print('\nbox total %%CPU: during goals mean %.0f (n=%d s) | launch up, no goal mean %.0f (n=%d s)'%(st.mean(inj),len(inj),st.mean(outj),len(outj)))
# per-process during goals vs not
pg=collections.Counter(); pn=collections.Counter(); ng=0; nn=0
for t in secs:
    if t<=secs[0]+20: continue
    g=t in goal_secs
    if g: ng+=1
    else: nn+=1
    for pid,(cpu,ncs,cmd) in rows[t].items(): (pg if g else pn)[cmd[:20]]+=cpu
print('%-22s %7s %7s'%('cmd','goal%','idle%'))
for c,_ in (pg+pn).most_common(14): print('%-22s %7.0f %7.0f'%(c,pg[c]/max(ng,1),pn[c]/max(nn,1)))
