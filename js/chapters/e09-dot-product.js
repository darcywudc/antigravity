/**
 * Chapter 9: Dot Products
 * Interactive visualization of dot product as "cooperation"
 */
window.chapterScripts['e09-dot-product'] = function () {
    console.log('Initializing Chapter 9: Dot Product');

    (function initDotProductDemo() {
        const canvas = document.getElementById('viz-dot-product');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 50;

        let v = { x: 2, y: 1 };
        let w = { x: 1, y: 2 };
        let dragging = null;

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function toWorld(canvasX, canvasY) {
            return {
                x: (canvasX - cx) / scale,
                y: -(canvasY - cy) / scale
            };
        }

        function dotProduct(a, b) {
            return a.x * b.x + a.y * b.y;
        }

        function magnitude(vec) {
            return Math.sqrt(vec.x * vec.x + vec.y * vec.y);
        }

        function drawArrow(from, to, color, label) {
            ctx.beginPath();
            ctx.moveTo(from.x, from.y);
            ctx.lineTo(to.x, to.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.stroke();

            // Arrowhead
            const angle = Math.atan2(to.y - from.y, to.x - from.x);
            const headLen = 12;
            ctx.beginPath();
            ctx.moveTo(to.x, to.y);
            ctx.lineTo(to.x - headLen * Math.cos(angle - 0.4), to.y - headLen * Math.sin(angle - 0.4));
            ctx.lineTo(to.x - headLen * Math.cos(angle + 0.4), to.y - headLen * Math.sin(angle + 0.4));
            ctx.closePath();
            ctx.fillStyle = color;
            ctx.fill();

            // Label
            ctx.fillStyle = color;
            ctx.font = 'bold 16px sans-serif';
            ctx.fillText(label, to.x + 10, to.y - 5);
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            // Grid
            ctx.strokeStyle = 'rgba(100,100,100,0.2)';
            ctx.lineWidth = 1;
            for (let i = -10; i <= 10; i++) {
                ctx.beginPath();
                ctx.moveTo(cx + i * scale, 0);
                ctx.lineTo(cx + i * scale, height);
                ctx.stroke();
                ctx.beginPath();
                ctx.moveTo(0, cy + i * scale);
                ctx.lineTo(width, cy + i * scale);
                ctx.stroke();
            }

            // Axes
            ctx.strokeStyle = 'rgba(100,100,100,0.5)';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(0, cy);
            ctx.lineTo(width, cy);
            ctx.moveTo(cx, 0);
            ctx.lineTo(cx, height);
            ctx.stroke();

            const origin = toCanvas(0, 0);
            const vEnd = toCanvas(v.x, v.y);
            const wEnd = toCanvas(w.x, w.y);

            // Draw vectors
            drawArrow(origin, vEnd, '#4a9eff', 'v');
            drawArrow(origin, wEnd, '#f97316', 'w');

            // Draw projection of v onto w
            const dot = dotProduct(v, w);
            const wMag = magnitude(w);
            const projScalar = dot / (wMag * wMag);
            const proj = { x: w.x * projScalar, y: w.y * projScalar };
            const projEnd = toCanvas(proj.x, proj.y);

            // Projection line (dashed)
            ctx.beginPath();
            ctx.setLineDash([5, 5]);
            ctx.moveTo(vEnd.x, vEnd.y);
            ctx.lineTo(projEnd.x, projEnd.y);
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
            ctx.lineWidth = 2;
            ctx.stroke();
            ctx.setLineDash([]);

            // Dot product result
            const dotVal = dot.toFixed(2);
            const dotColor = dot > 0 ? '#10b981' : dot < 0 ? '#ef4444' : '#888';

            // Display
            const display = document.getElementById('dot-result');
            if (display) {
                display.innerHTML = `v · w = <span style="color:${dotColor}; font-weight:bold; font-size:1.5rem;">${dotVal}</span>`;
            }

            const interpretation = document.getElementById('dot-interpretation');
            if (interpretation) {
                if (dot > 0.1) {
                    interpretation.textContent = '✅ 大致同向 (合作)';
                    interpretation.style.color = '#10b981';
                } else if (dot < -0.1) {
                    interpretation.textContent = '❌ 大致反向 (对抗)';
                    interpretation.style.color = '#ef4444';
                } else {
                    interpretation.textContent = '⭕ 垂直 (无关)';
                    interpretation.style.color = '#f97316';
                }
            }

            // Drag handles
            ctx.beginPath();
            ctx.arc(vEnd.x, vEnd.y, 10, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(74, 158, 255, 0.3)';
            ctx.fill();
            ctx.beginPath();
            ctx.arc(wEnd.x, wEnd.y, 10, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(249, 115, 22, 0.3)';
            ctx.fill();
        }

        canvas.addEventListener('mousedown', (e) => {
            const rect = canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            const my = e.clientY - rect.top;

            const vEnd = toCanvas(v.x, v.y);
            const wEnd = toCanvas(w.x, w.y);

            if (Math.hypot(mx - vEnd.x, my - vEnd.y) < 15) dragging = 'v';
            else if (Math.hypot(mx - wEnd.x, my - wEnd.y) < 15) dragging = 'w';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            const rect = canvas.getBoundingClientRect();
            const world = toWorld(e.clientX - rect.left, e.clientY - rect.top);

            if (dragging === 'v') {
                v.x = world.x;
                v.y = world.y;
            } else if (dragging === 'w') {
                w.x = world.x;
                w.y = world.y;
            }
            draw();
        });

        canvas.addEventListener('mouseup', () => dragging = null);
        canvas.addEventListener('mouseleave', () => dragging = null);

        draw();
    })();
};
