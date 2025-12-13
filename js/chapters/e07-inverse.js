/**
 * Chapter 7: Inverse Matrices
 * Interactive demonstration of forward and inverse transformations
 */
window.chapterScripts['e07-inverse'] = function () {
    console.log('Initializing Chapter 7: Inverse Matrices');

    (function initInverseDemo() {
        const canvas = document.getElementById('viz-inverse');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 50;

        let showInverse = false;
        let animationProgress = 0;
        let animating = false;

        // Matrix A
        const A = { a: 2, b: 1, c: 0, d: 1.5 };

        function computeInverse(m) {
            const det = m.a * m.d - m.b * m.c;
            if (Math.abs(det) < 0.001) return null;
            return {
                a: m.d / det,
                b: -m.b / det,
                c: -m.c / det,
                d: m.a / det
            };
        }

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function transform(x, y, m, t = 1) {
            // Interpolate from identity to m
            const a = 1 + (m.a - 1) * t;
            const b = m.b * t;
            const c = m.c * t;
            const d = 1 + (m.d - 1) * t;
            return {
                x: a * x + b * y,
                y: c * x + d * y
            };
        }

        function drawArrow(from, to, color) {
            ctx.beginPath();
            ctx.moveTo(from.x, from.y);
            ctx.lineTo(to.x, to.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 3;
            ctx.stroke();

            // Arrowhead
            const angle = Math.atan2(to.y - from.y, to.x - from.x);
            const headLen = 10;
            ctx.beginPath();
            ctx.moveTo(to.x, to.y);
            ctx.lineTo(to.x - headLen * Math.cos(angle - 0.4), to.y - headLen * Math.sin(angle - 0.4));
            ctx.lineTo(to.x - headLen * Math.cos(angle + 0.4), to.y - headLen * Math.sin(angle + 0.4));
            ctx.closePath();
            ctx.fillStyle = color;
            ctx.fill();
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

            const origin = toCanvas(0, 0);
            const matrix = showInverse ? computeInverse(A) : A;
            if (!matrix) {
                ctx.fillStyle = '#ef4444';
                ctx.font = 'bold 20px sans-serif';
                ctx.fillText('No inverse (det = 0)!', 20, 40);
                return;
            }

            // Draw transformed basis vectors
            const iHat = transform(1, 0, matrix, animationProgress);
            const jHat = transform(0, 1, matrix, animationProgress);

            const iEnd = toCanvas(iHat.x, iHat.y);
            const jEnd = toCanvas(jHat.x, jHat.y);

            drawArrow(origin, iEnd, '#f97316');
            drawArrow(origin, jEnd, '#10b981');

            // Labels
            ctx.fillStyle = '#f97316';
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText('î', iEnd.x + 5, iEnd.y - 5);
            ctx.fillStyle = '#10b981';
            ctx.fillText('ĵ', jEnd.x + 5, jEnd.y - 5);

            // Status
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 16px sans-serif';
            ctx.fillText(showInverse ? '逆变换 A⁻¹' : '原变换 A', 20, 30);
        }

        function animate() {
            if (!animating) return;
            animationProgress += 0.05;
            if (animationProgress >= 1) {
                animationProgress = 1;
                animating = false;
            }
            draw();
            if (animating) requestAnimationFrame(animate);
        }

        // Buttons
        const applyBtn = document.getElementById('btn-apply-transform');
        const inverseBtn = document.getElementById('btn-apply-inverse');
        const resetBtn = document.getElementById('btn-reset-inverse');

        if (applyBtn) {
            applyBtn.addEventListener('click', () => {
                showInverse = false;
                animationProgress = 0;
                animating = true;
                animate();
            });
        }

        if (inverseBtn) {
            inverseBtn.addEventListener('click', () => {
                showInverse = true;
                animationProgress = 0;
                animating = true;
                animate();
            });
        }

        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                animationProgress = 0;
                showInverse = false;
                draw();
            });
        }

        draw();
    })();
};
