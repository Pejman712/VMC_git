"use strict";
/* Shared behaviour for the VMC pages: a scan-to-open QR code and a
   multi-option colour-theme menu. Loaded on every page together with
   assets/qrcode.min.js (which defines the global `qrcode`). */
(function () {

  /* ---------------- component styles (injected once) ---------------- */
  var CSS = [
    '.qr-band{background:var(--panel);border-bottom:1px solid var(--border)}',
    '.qr-band-in{max-width:var(--maxw,1140px);margin:0 auto;padding:16px 22px;display:flex;align-items:center;gap:20px;flex-wrap:wrap}',
    '.qr-card{background:#fff;border-radius:14px;padding:10px;box-shadow:0 1px 2px rgba(16,32,54,.14),0 8px 24px rgba(16,32,54,.16);flex:none;line-height:0}',
    '.qr-card svg{display:block;width:132px;height:132px}',
    '.qr-copy{min-width:0}',
    '.qr-copy .qr-kicker{font-family:var(--mono);font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--accent-ink)}',
    '.qr-copy h2{font-family:var(--display);font-size:19px;margin:3px 0 4px;color:var(--ink)}',
    '.qr-copy p{margin:0;color:var(--muted);font-size:13.5px;max-width:52ch}',
    '.qr-copy .qr-url{font-family:var(--mono);font-size:12px;color:var(--ink-2);word-break:break-all;margin-top:6px}',
    '@media (max-width:560px){.qr-card svg{width:104px;height:104px}}',
    /* theme menu */
    '.theme-menu{position:fixed;z-index:200;background:var(--surface);border:1px solid var(--border-2);border-radius:12px;box-shadow:var(--shadow);padding:6px;min-width:186px}',
    '.theme-menu[hidden]{display:none}',
    '.theme-menu .th-head{font-family:var(--mono);font-size:10.5px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);padding:6px 8px 4px}',
    '.theme-opt{display:flex;align-items:center;gap:10px;width:100%;border:0;background:transparent;color:var(--ink);font:inherit;font-size:14px;text-align:left;padding:8px 9px;border-radius:8px;cursor:pointer}',
    '.theme-opt:hover,.theme-opt:focus-visible{background:var(--panel-2)}',
    '.theme-opt .sw{width:18px;height:18px;border-radius:5px;border:1px solid var(--border-2);flex:none}',
    '.theme-opt .ck{margin-left:auto;color:var(--accent);opacity:0}',
    '.theme-opt[aria-checked="true"] .ck{opacity:1}',
    '.theme-opt[aria-checked="true"]{font-weight:600}'
  ].join('');
  var st = document.createElement('style'); st.textContent = CSS;
  document.head.appendChild(st);

  /* ---------------- QR code ---------------- */
  function buildQR() {
    var box = document.getElementById('qrCode');
    if (!box || typeof qrcode === 'undefined') return;
    var url = location.href;
    var urlEl = document.getElementById('qrUrl');
    if (urlEl) urlEl.textContent = url.replace(/^https?:\/\//, '');
    try {
      var qr = qrcode(0, 'M');
      qr.addData(url);
      qr.make();
      var n = qr.getModuleCount(), m = 2, size = n + m * 2, rects = '';
      for (var r = 0; r < n; r++) {
        for (var c = 0; c < n; c++) {
          if (qr.isDark(r, c)) rects += '<rect x="' + (c + m) + '" y="' + (r + m) + '" width="1" height="1"/>';
        }
      }
      box.innerHTML = '<svg viewBox="0 0 ' + size + ' ' + size + '" xmlns="http://www.w3.org/2000/svg" ' +
        'shape-rendering="crispEdges" role="img" aria-label="QR code linking to this page">' +
        '<rect width="' + size + '" height="' + size + '" fill="#ffffff"/>' +
        '<g fill="#0b1220">' + rects + '</g></svg>';
    } catch (e) {
      box.textContent = 'QR unavailable';
    }
  }

  /* ---------------- theme menu ---------------- */
  var THEMES = [
    { id: 'system',   label: 'System',        sw: 'linear-gradient(135deg,#EEF2F7 0 50%,#0C1420 50% 100%)' },
    { id: 'light',    label: 'Light',         sw: '#F1F4F8' },
    { id: 'dark',     label: 'Dark',          sw: '#0C1420' },
    { id: 'midnight', label: 'Midnight',      sw: '#070B14' },
    { id: 'sepia',    label: 'Sepia',         sw: '#F3EBDD' },
    { id: 'contrast', label: 'High contrast', sw: 'linear-gradient(135deg,#000 0 50%,#fff 50% 100%)' }
  ];

  function current() {
    try { return localStorage.getItem('ros-theme') || 'system'; } catch (e) { return 'system'; }
  }
  function apply(id) {
    var el = document.documentElement;
    if (id === 'system') el.removeAttribute('data-theme');
    else el.setAttribute('data-theme', id);
    try {
      if (id === 'system') localStorage.removeItem('ros-theme');
      else localStorage.setItem('ros-theme', id);
    } catch (e) {}
    document.dispatchEvent(new CustomEvent('themechange', { detail: { theme: id } }));
    if (typeof window.redrawAll === 'function') { try { window.redrawAll(); } catch (e) {} }
  }

  function initThemeMenu() {
    var btn = document.getElementById('themeBtn');
    if (!btn) return;
    btn.setAttribute('aria-haspopup', 'true');
    btn.setAttribute('aria-expanded', 'false');

    var menu = document.createElement('div');
    menu.className = 'theme-menu';
    menu.setAttribute('role', 'menu');
    menu.hidden = true;
    menu.innerHTML = '<div class="th-head">Colour theme</div>';
    THEMES.forEach(function (t) {
      var o = document.createElement('button');
      o.type = 'button';
      o.className = 'theme-opt';
      o.setAttribute('role', 'menuitemradio');
      o.dataset.theme = t.id;
      o.innerHTML = '<span class="sw" style="background:' + t.sw + '"></span>' +
        '<span>' + t.label + '</span><span class="ck" aria-hidden="true">✓</span>';
      o.addEventListener('click', function () { apply(t.id); mark(); close(); btn.focus(); });
      menu.appendChild(o);
    });
    document.body.appendChild(menu);

    function mark() {
      var c = current();
      menu.querySelectorAll('.theme-opt').forEach(function (o) {
        o.setAttribute('aria-checked', o.dataset.theme === c ? 'true' : 'false');
      });
    }
    function place() {
      var r = btn.getBoundingClientRect();
      menu.style.top = (r.bottom + 8) + 'px';
      var right = Math.max(8, window.innerWidth - r.right);
      menu.style.right = right + 'px';
      menu.style.left = 'auto';
    }
    function open() { mark(); menu.hidden = false; place(); btn.setAttribute('aria-expanded', 'true'); }
    function close() { menu.hidden = true; btn.setAttribute('aria-expanded', 'false'); }
    function toggle() { menu.hidden ? open() : close(); }

    btn.addEventListener('click', function (e) { e.stopPropagation(); toggle(); });
    document.addEventListener('click', function (e) {
      if (!menu.hidden && !menu.contains(e.target) && e.target !== btn) close();
    });
    window.addEventListener('resize', function () { if (!menu.hidden) place(); });
    window.addEventListener('scroll', function () { if (!menu.hidden) place(); }, true);
    document.addEventListener('keydown', function (e) {
      if (e.key === 'Escape' && !menu.hidden) { close(); btn.focus(); return; }
      if ((e.key === 't' || e.key === 'T') && !/input|textarea|select/i.test(e.target.tagName)) {
        var ids = THEMES.map(function (t) { return t.id; });
        apply(ids[(ids.indexOf(current()) + 1) % ids.length]); mark();
      }
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function () { buildQR(); initThemeMenu(); });
  } else {
    buildQR(); initThemeMenu();
  }
})();
