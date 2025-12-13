/**
 * Chapter 14: Eigenvectors and Eigenvalues
 * Interactive visualization showing eigenvectors as "stubborn directions"
 */
window.chapterScripts['e14-eigenvectors'] = function () {
    console.log('Initializing Chapter 14: Eigenvectors');

    (function initEigenDemo() {
        const canvas = document.getElementById('viz-eigen');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 40;

        // Matrix with known eigenvectors
        // A = [[3, 1], [0, 2]]
        // Eigenvalues: 3, 2
        // Eigenvector for λ=3: (1, 0)
        // Eigenvector for λ=2: (-1, 1)
        let A = { a: 3, b: 1, c: 0, d: 2 };
        let animProgress = 0;
        let animating = false;

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function transform(x, y, t = 1) {
            // Interpolate from identity to A
            const a = 1 + (A.a - 1) * t;
            const b = A.b * t;
            const c = A.c * t;
            const d = 1 + (A.d - 1) * t;
            return { x: a * x + b * y, y: c * x + d * y };
        }

        function drawArrow(from, to, color, lineWidth = 3) {
            ctx.beginPath();
            ctx.moveTo(from.x, from.y);
            ctx.lineTo(to.x, to.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = lineWidth;
            ctx.stroke();

            const angle = Math.atan2(to.y - from.y, to.x - from.x);
            const headLen = 8;
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

            // Draw multiple test vectors
            const testVectors = [
                { x: 1.5, y: 0.5, color: '#888' },   // Not eigenvector
                { x: 0.5, y: 1.5, color: '#888' },   // Not eigenvector
                { x: 1, y: 0, color: '#10b981' },    // Eigenvector (λ=3)
                { x: -1, y: 1, color: '#f97316' },   // Eigenvector (λ=2)
            ];

            // Draw original vectors (faded)
            testVectors.forEach((v, i) => {
                const end = toCanvas(v.x, v.y);
                ctx.globalAlpha = 0.3;
                drawArrow(origin, end, v.color, 2);
                ctx.globalAlpha = 1;
            });

            // Draw transformed vectors
            testVectors.forEach((v, i) => {
                const transformed = transform(v.x, v.y, animProgress);
                const end = toCanvas(transformed.x, transformed.y);
                drawArrow(origin, end, v.color, 3);

                // Label eigenvectors
                if (i === 2) {
                    ctx.fillStyle = '#10b981';
                    ctx.font = 'bold 12px sans-serif';
                    ctx.fillText('特征向量 λ=3', end.x + 5, end.y - 5);
                }
                if (i === 3) {
                    ctx.fillStyle = '#f97316';
                    ctx.font = 'bold 12px sans-serif';
                    ctx.fillText('特征向量 λ=2', end.x + 5, end.y + 15);
                }
            });

            // Info
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText('绿色和橙色向量是特征向量 —— 方向不变！', 20, 30);
            ctx.fillStyle = '#888';
            ctx.font = '12px sans-serif';
            ctx.fillText('灰色向量方向改变了', 20, 50);
        }

        function animate() {
            if (!animating) return;
            animProgress += 0.02;
            if (animProgress >= 1) {
                animProgress = 1;
                animating = false;
            }
            draw();
            if (animating) requestAnimationFrame(animate);
        }

        const transformBtn = document.getElementById('btn-eigen-transform');
        const resetBtn = document.getElementById('btn-eigen-reset');

        if (transformBtn) {
            transformBtn.addEventListener('click', () => {
                animProgress = 0;
                animating = true;
                animate();
            });
        }

        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                animProgress = 0;
                animating = false;
                draw();
            });
        }

        draw();
    })();
};
