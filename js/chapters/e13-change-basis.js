/**
 * Chapter 13: Change of Basis
 * Interactive visualization of coordinate transformation between bases
 */
window.chapterScripts['e13-change-basis'] = function () {
    console.log('Initializing Chapter 13: Change of Basis');

    (function initBasisChangeDemo() {
        const canvas = document.getElementById('viz-change-basis');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 40;

        // New basis vectors
        let b1 = { x: 2, y: 1 };
        let b2 = { x: -1, y: 1 };
        let showNewGrid = true;
        let dragging = null;

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function toWorld(cx, cy) {
            return { x: (cx - width / 2) / scale, y: -(cy - height / 2) / scale };
        }

        function drawArrow(from, to, color, label) {
            ctx.beginPath();
            ctx.moveTo(from.x, from.y);
            ctx.lineTo(to.x, to.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.stroke();

            const angle = Math.atan2(to.y - from.y, to.x - from.x);
            const headLen = 10;
            ctx.beginPath();
            ctx.moveTo(to.x, to.y);
            ctx.lineTo(to.x - headLen * Math.cos(angle - 0.4), to.y - headLen * Math.sin(angle - 0.4));
            ctx.lineTo(to.x - headLen * Math.cos(angle + 0.4), to.y - headLen * Math.sin(angle + 0.4));
            ctx.closePath();
            ctx.fillStyle = color;
            ctx.fill();

            if (label) {
                ctx.fillStyle = color;
                ctx.font = 'bold 14px sans-serif';
                ctx.fillText(label, to.x + 8, to.y - 5);
            }
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            const origin = toCanvas(0, 0);

            // Standard grid (faded)
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

            // New basis grid
            if (showNewGrid) {
                ctx.strokeStyle = 'rgba(16, 185, 129, 0.15)';
                ctx.lineWidth = 1;
                for (let i = -5; i <= 5; i++) {
                    for (let j = -5; j <= 5; j++) {
                        // Lines along b1 direction
                        const startX = i * b1.x + j * b2.x;
                        const startY = i * b1.y + j * b2.y;
                        const endX = startX + b1.x * 5;
                        const endY = startY + b1.y * 5;
                        const p1 = toCanvas(startX - b1.x * 5, startY - b1.y * 5);
                        const p2 = toCanvas(endX, endY);
                        ctx.beginPath();
                        ctx.moveTo(p1.x, p1.y);
                        ctx.lineTo(p2.x, p2.y);
                        ctx.stroke();
                    }
                }
            }

            // Standard basis (faded)
            const iHat = toCanvas(1, 0);
            const jHat = toCanvas(0, 1);
            drawArrow(origin, iHat, 'rgba(150,150,150,0.5)', 'î');
            drawArrow(origin, jHat, 'rgba(150,150,150,0.5)', 'ĵ');

            // New basis vectors
            const b1End = toCanvas(b1.x, b1.y);
            const b2End = toCanvas(b2.x, b2.y);
            drawArrow(origin, b1End, '#f97316', 'b₁');
            drawArrow(origin, b2End, '#4a9eff', 'b₂');

            // A sample point
            const newCoord = { a: 1.5, b: 1 }; // In new basis
            const worldX = newCoord.a * b1.x + newCoord.b * b2.x;
            const worldY = newCoord.a * b1.y + newCoord.b * b2.y;
            const pointCanvas = toCanvas(worldX, worldY);

            ctx.beginPath();
            ctx.arc(pointCanvas.x, pointCanvas.y, 8, 0, Math.PI * 2);
            ctx.fillStyle = '#ec4899';
            ctx.fill();

            // Info
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText(`新基下坐标: (${newCoord.a}, ${newCoord.b})`, 20, 30);
            ctx.fillText(`标准基坐标: (${worldX.toFixed(2)}, ${worldY.toFixed(2)})`, 20, 50);

            // Drag handles
            ctx.beginPath();
            ctx.arc(b1End.x, b1End.y, 8, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(249, 115, 22, 0.3)';
            ctx.fill();
            ctx.beginPath();
            ctx.arc(b2End.x, b2End.y, 8, 0, Math.PI * 2);
            ctx.fillStyle = 'rgba(74, 158, 255, 0.3)';
            ctx.fill();
        }

        canvas.addEventListener('mousedown', (e) => {
            const rect = canvas.getBoundingClientRect();
            const mx = e.clientX - rect.left;
            const my = e.clientY - rect.top;

            const b1End = toCanvas(b1.x, b1.y);
            const b2End = toCanvas(b2.x, b2.y);

            if (Math.hypot(mx - b1End.x, my - b1End.y) < 12) dragging = 'b1';
            else if (Math.hypot(mx - b2End.x, my - b2End.y) < 12) dragging = 'b2';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            const rect = canvas.getBoundingClientRect();
            const world = toWorld(e.clientX - rect.left, e.clientY - rect.top);

            if (dragging === 'b1') {
                b1.x = world.x;
                b1.y = world.y;
            } else {
                b2.x = world.x;
                b2.y = world.y;
            }
            draw();
        });

        canvas.addEventListener('mouseup', () => dragging = null);
        canvas.addEventListener('mouseleave', () => dragging = null);

        draw();
    })();
};
