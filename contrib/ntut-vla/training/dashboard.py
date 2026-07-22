"""
Live training dashboard — http://127.0.0.1:8085

Single-file server: serves an auto-refreshing HTML page + /status (the JSON
written by auto_research) + /gpu (live nvidia-smi numbers). No dependencies
beyond the standard library.

Run:  python training/dashboard.py
"""
from __future__ import annotations

import json
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

STATUS = Path(__file__).resolve().parent / "status.json"
PORT = 8085

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>Auto-Research Monitor</title>
<style>
 body{font-family:Segoe UI,system-ui,sans-serif;background:#10151a;color:#dde;margin:0;padding:24px}
 h1{font-size:20px;margin:0 0 4px} .sub{color:#8aa;font-size:12px;margin-bottom:18px}
 .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:12px;margin-bottom:18px}
 .card{background:#1a2129;border:1px solid #2a3542;border-radius:10px;padding:14px}
 .k{color:#8aa;font-size:11px;text-transform:uppercase;letter-spacing:1px}
 .v{font-size:26px;font-weight:600;margin-top:4px}
 .stage{color:#35c3d8} .ok{color:#4dd67a} .bad{color:#ff6b6b}
 canvas{width:100%;height:180px;background:#1a2129;border:1px solid #2a3542;border-radius:10px}
 table{width:100%;border-collapse:collapse;margin-top:16px;font-size:12.5px}
 th,td{padding:6px 10px;border-bottom:1px solid #2a3542;text-align:left}
 th{color:#8aa;font-weight:600} .bar{height:8px;background:#243040;border-radius:4px;overflow:hidden;margin-top:8px}
 .fill{height:100%;background:#35c3d8;transition:width .5s}
</style></head><body>
<h1>Auto-Research Monitor <span id="stage" class="stage"></span></h1>
<div class="sub">started <span id="started"></span> · updated <span id="updated"></span> · device <span id="device"></span></div>
<div class="grid">
 <div class="card"><div class="k">Iteration</div><div class="v" id="iter">–</div><div class="k" id="cfg"></div></div>
 <div class="card"><div class="k">Dataset</div><div class="v" id="gen">–</div><div class="bar"><div class="fill" id="genbar" style="width:0%"></div></div></div>
 <div class="card"><div class="k">Epoch</div><div class="v" id="epoch">–</div><div class="bar"><div class="fill" id="epbar" style="width:0%"></div></div></div>
 <div class="card"><div class="k">Val loss (best)</div><div class="v" id="val">–</div></div>
 <div class="card"><div class="k">GPU util / mem / temp</div><div class="v" id="gpu">–</div></div>
</div>
<canvas id="chart" width="1200" height="180"></canvas>
<div class="grid" style="margin-top:18px">
 <div class="card"><div class="k">Reach rate (no shield)</div><div class="v" id="reach">–</div></div>
 <div class="card"><div class="k">NFZ entry (no shield)</div><div class="v" id="entry">–</div></div>
 <div class="card"><div class="k">Shield interventions</div><div class="v" id="inter">–</div></div>
 <div class="card"><div class="k">Verdict</div><div class="v" id="verdict" style="font-size:16px">–</div></div>
</div>
<table id="tbl"><thead><tr><th>it</th><th>config</th><th>val MSE</th><th>reach</th><th>NFZ entry</th><th>interv.</th><th>time</th><th>wall</th><th>verdict</th></tr></thead><tbody></tbody></table>
<script>
async function tick(){
 try{
  const s = await (await fetch('/status')).json();
  const g = await (await fetch('/gpu')).json();
  document.getElementById('stage').textContent = '· ' + s.stage + (s.done ? ' ✔' : '');
  document.getElementById('started').textContent = s.run_started;
  document.getElementById('updated').textContent = s.updated;
  document.getElementById('device').textContent = s.device || '';
  document.getElementById('iter').textContent = s.iteration;
  document.getElementById('cfg').textContent = s.config ? JSON.stringify(s.config) : '';
  if(s.gen && s.gen.total){
    document.getElementById('gen').textContent = s.gen.done + '/' + s.gen.total + ' eps · ' + (s.gen.samples||0).toLocaleString() + ' samples';
    document.getElementById('genbar').style.width = (100*s.gen.done/s.gen.total)+'%';
  }
  if(s.train && s.train.epochs){
    document.getElementById('epoch').textContent = (s.train.epoch||0) + '/' + s.train.epochs;
    document.getElementById('epbar').style.width = (100*(s.train.epoch||0)/s.train.epochs)+'%';
    document.getElementById('val').textContent = (s.train.val_loss??'–') + ' (' + (s.train.best_val??'–') + ')';
  }
  document.getElementById('gpu').textContent = g.util + '% / ' + g.mem + ' / ' + g.temp + '°C';
  if(s.eval && s.eval.reach_rate !== undefined){
    document.getElementById('reach').textContent = (100*s.eval.reach_rate).toFixed(1)+'%';
    document.getElementById('entry').textContent = (100*s.eval.nfz_entry_rate).toFixed(1)+'%';
    document.getElementById('inter').textContent = (100*s.eval.intervention_rate).toFixed(1)+'%';
  }
  const vd = document.getElementById('verdict');
  vd.textContent = s.verdict || (s.done ? 'done' : 'running...');
  vd.className = 'v ' + (s.verdict && s.verdict.includes('PASS') ? 'ok' : '');
  // loss chart
  const h = (s.train && s.train.loss_hist) || [];
  const c = document.getElementById('chart'), ctx = c.getContext('2d');
  ctx.clearRect(0,0,c.width,c.height);
  if(h.length > 1){
    const mn = Math.min(...h), mx = Math.max(...h);
    ctx.strokeStyle = '#35c3d8'; ctx.lineWidth = 2; ctx.beginPath();
    h.forEach((v,i)=>{
      const x = 30 + i*(c.width-50)/(h.length-1);
      const y = 15 + (c.height-40)*(1-(v-mn)/(mx-mn+1e-9));
      i ? ctx.lineTo(x,y) : ctx.moveTo(x,y);
    });
    ctx.stroke();
    ctx.fillStyle = '#8aa'; ctx.font = '11px sans-serif';
    ctx.fillText('val loss: ' + mx.toFixed(5) + ' → ' + mn.toFixed(5) + ' (last ' + h.length + ' epochs)', 30, c.height-8);
  }
  // iterations table
  const tb = document.querySelector('#tbl tbody'); tb.innerHTML = '';
  (s.iterations||[]).forEach(r=>{
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${r.it}</td><td>${r.cfg}</td><td>${r.val}</td>`+
      `<td>${(100*r.reach_rate).toFixed(1)}%</td><td>${(100*r.nfz_entry_rate).toFixed(1)}%</td>`+
      `<td>${(100*r.intervention_rate).toFixed(1)}%</td><td>${r.mean_time_s.toFixed(0)}s</td>`+
      `<td>${r.wall_s}s</td><td class="${r.verdict.includes('PASS')?'ok':'bad'}">${r.verdict}</td>`;
    tb.appendChild(tr);
  });
 }catch(e){ document.getElementById('stage').textContent = '· waiting for status...'; }
}
setInterval(tick, 2000); tick();
</script></body></html>"""


def gpu_stats() -> dict:
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5).stdout.strip()
        util, mu, mt, temp = [x.strip() for x in out.split(",")]
        return {"util": util, "mem": f"{int(mu) / 1024:.1f}/{int(mt) / 1024:.0f} GB",
                "temp": temp}
    except Exception:                                    # noqa: BLE001
        return {"util": "?", "mem": "?", "temp": "?"}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):                           # silence request spam
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/status":
            body = STATUS.read_bytes() if STATUS.exists() else b"{}"
            self._send(200, "application/json", body)
        elif self.path == "/gpu":
            self._send(200, "application/json", json.dumps(gpu_stats()).encode())
        else:
            self._send(200, "text/html; charset=utf-8", PAGE.encode())


if __name__ == "__main__":
    print(f"dashboard: http://127.0.0.1:{PORT}")
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
