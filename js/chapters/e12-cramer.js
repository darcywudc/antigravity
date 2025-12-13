/**
 * Chapter 12: Cramer's Rule
 * Interactive visualization of area ratios for solving equations
 */
window.chapterScripts['e12-cramer'] = function () {
    console.log('Initializing Chapter 12: Cramer\'s Rule');

    (function initCramerDemo() {
        const canvas = document.getElementById('viz-cramer');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const cx = width / 2;
        const cy = height / 2;
        const scale = 40;

        // Matrix A and vector b
        let A = { a: 2, b: 1, c: 1, d: 3 };
        let b = { x: 5, y: 5 };

        function toCanvas(x, y) {
            return { x: cx + x * scale, y: cy - y * scale };
        }

        function det2x2(a, b, c, d) {
            return a * d - b * c;
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);

            // Grid
            ctx.strokeStyle = 'rgba(100,100,100,0.2)';
            ctx.lineWidth = 1;
            for (let i = -10; i <= 10; i++) {
                const p = toCanvas(i, 0);
                ctx.beginPath();
                ctx.moveTo(p.x, 0);
                ctx.lineTo(p.x, height);
                ctx.stroke();
                const q = toCanvas(0, i);
                ctx.beginPath();
                ctx.moveTo(0, q.y);
                ctx.lineTo(width, q.y);
                ctx.stroke();
            }

            const origin = toCanvas(0, 0);
            const col1 = toCanvas(A.a, A.c); // First column of A
            const col2 = toCanvas(A.b, A.d); // Second column of A
            const bPos = toCanvas(b.x, b.y);

            // Draw column vectors of A
            ctx.lineWidth = 3;
            ctx.strokeStyle = '#4a9eff';
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(col1.x, col1.y);
            ctx.stroke();

            ctx.strokeStyle = '#f97316';
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(col2.x, col2.y);
            ctx.stroke();

            // Draw b vector
            ctx.strokeStyle = '#10b981';
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(bPos.x, bPos.y);
            ctx.stroke();

            // Draw parallelogram (original)
            const corner = toCanvas(A.a + A.b, A.c + A.d);
            ctx.fillStyle = 'rgba(139, 92, 246, 0.2)';
            ctx.beginPath();
            ctx.moveTo(origin.x, origin.y);
            ctx.lineTo(col1.x, col1.y);
            ctx.lineTo(corner.x, corner.y);
            ctx.lineTo(col2.x, col2.y);
            ctx.closePath();
            ctx.fill();

            // Calculate solution using Cramer's rule
            const detA = det2x2(A.a, A.b, A.c, A.d);
            const detAx = det2x2(b.x, A.b, b.y, A.d);
            const detAy = det2x2(A.a, b.x, A.c, b.y);

            let x = 0, y = 0;
            if (Math.abs(detA) > 0.001) {
                x = detAx / detA;
                y = detAy / detA;
            }

            // Labels
            ctx.font = 'bold 14px sans-serif';
            ctx.fillStyle = '#4a9eff';
            ctx.fillText('A₁', col1.x + 5, col1.y - 5);
            ctx.fillStyle = '#f97316';
            ctx.fillText('A₂', col2.x + 5, col2.y - 5);
            ctx.fillStyle = '#10b981';
            ctx.fillText('b', bPos.x + 5, bPos.y - 5);

            // Solution display
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 16px sans-serif';
            ctx.fillText(`det(A) = ${detA.toFixed(2)}`, 20, 30);
            ctx.fillText(`x = ${x.toFixed(2)}, y = ${y.toFixed(2)}`, 20, 55);

            // Update input displays
            const xDisplay = document.getElementById('cramer-x');
            const yDisplay = document.getElementById('cramer-y');
            if (xDisplay) xDisplay.textContent = x.toFixed(3);
            if (yDisplay) yDisplay.textContent = y.toFixed(3);
        }

        // Slider bindings
        ['cramer-b1', 'cramer-b2'].forEach((id, i) => {
            const slider = document.getElementById(id);
            if (slider) {
                slider.addEventListener('input', (e) => {
                    const val = parseFloat(e.target.value);
                    if (i === 0) b.x = val;
                    else b.y = val;
                    draw();
                });
            }
        });

        draw();
    })();
};
