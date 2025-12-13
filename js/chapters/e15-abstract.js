/**
 * Chapter 15: Abstract Vector Spaces
 * Interactive demonstration that functions can be vectors
 */
window.chapterScripts['e15-abstract'] = function () {
    console.log('Initializing Chapter 15: Abstract Vector Spaces');

    (function initFunctionSpaceDemo() {
        const canvas = document.getElementById('viz-function-space');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;
        const padding = 50;

        let aCoeff = 1;
        let bCoeff = 1;

        function f1(x) { return Math.sin(x); }
        function f2(x) { return Math.cos(x); }

        function drawAxes() {
            ctx.strokeStyle = 'rgba(100,100,100,0.5)';
            ctx.lineWidth = 1;
            // X axis
            ctx.beginPath();
            ctx.moveTo(padding, height / 2);
            ctx.lineTo(width - padding, height / 2);
            ctx.stroke();
            // Y axis
            ctx.beginPath();
            ctx.moveTo(padding, padding);
            ctx.lineTo(padding, height - padding);
            ctx.stroke();

            // Labels
            ctx.fillStyle = '#888';
            ctx.font = '12px sans-serif';
            ctx.fillText('0', padding - 10, height / 2 + 15);
            ctx.fillText('2π', width - padding - 10, height / 2 + 15);
        }

        function drawFunction(fn, color, label) {
            const xMin = 0;
            const xMax = 2 * Math.PI;
            const yScale = 80;

            ctx.beginPath();
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;

            for (let px = padding; px <= width - padding; px++) {
                const x = xMin + (px - padding) / (width - 2 * padding) * (xMax - xMin);
                const y = fn(x);
                const py = height / 2 - y * yScale;

                if (px === padding) {
                    ctx.moveTo(px, py);
                } else {
                    ctx.lineTo(px, py);
                }
            }
            ctx.stroke();

            // Label
            ctx.fillStyle = color;
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText(label, width - padding + 10, height / 2 - fn(2 * Math.PI) * 80);
        }

        function draw() {
            ctx.clearRect(0, 0, width, height);
            drawAxes();

            // Draw individual functions (faded)
            ctx.globalAlpha = 0.3;
            drawFunction(f1, '#4a9eff', 'sin(x)');
            drawFunction(f2, '#f97316', 'cos(x)');
            ctx.globalAlpha = 1;

            // Draw linear combination
            const combined = (x) => aCoeff * f1(x) + bCoeff * f2(x);
            drawFunction(combined, '#10b981', `${aCoeff}sin + ${bCoeff}cos`);

            // Title
            ctx.fillStyle = '#fff';
            ctx.font = 'bold 16px sans-serif';
            ctx.fillText('函数也是向量！', 20, 30);
            ctx.font = '14px sans-serif';
            ctx.fillStyle = '#888';
            ctx.fillText(`f(x) = ${aCoeff.toFixed(1)}·sin(x) + ${bCoeff.toFixed(1)}·cos(x)`, 20, 55);
        }

        // Sliders
        const sliderA = document.getElementById('func-coeff-a');
        const sliderB = document.getElementById('func-coeff-b');

        if (sliderA) {
            sliderA.addEventListener('input', (e) => {
                aCoeff = parseFloat(e.target.value);
                const label = document.getElementById('coeff-a-val');
                if (label) label.textContent = aCoeff.toFixed(1);
                draw();
            });
        }

        if (sliderB) {
            sliderB.addEventListener('input', (e) => {
                bCoeff = parseFloat(e.target.value);
                const label = document.getElementById('coeff-b-val');
                if (label) label.textContent = bCoeff.toFixed(1);
                draw();
            });
        }

        draw();
    })();

    // Color mixing demo
    (function initColorDemo() {
        const canvas = document.getElementById('viz-color-vectors');
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const width = canvas.width = canvas.offsetWidth;
        const height = canvas.height = canvas.offsetHeight;

        let r = 150, g = 100, b = 200;

        function draw() {
            ctx.clearRect(0, 0, width, height);

            const boxSize = 80;
            const spacing = 30;
            const startX = 30;
            const y = height / 2 - boxSize / 2;

            // R component
            ctx.fillStyle = `rgb(${r}, 0, 0)`;
            ctx.fillRect(startX, y, boxSize, boxSize);
            ctx.fillStyle = '#fff';
            ctx.font = '14px sans-serif';
            ctx.fillText(`R=${r}`, startX + 20, y + boxSize + 20);

            // + sign
            ctx.fillStyle = '#888';
            ctx.font = 'bold 24px sans-serif';
            ctx.fillText('+', startX + boxSize + 10, y + boxSize / 2 + 8);

            // G component
            const gX = startX + boxSize + spacing + 20;
            ctx.fillStyle = `rgb(0, ${g}, 0)`;
            ctx.fillRect(gX, y, boxSize, boxSize);
            ctx.fillStyle = '#fff';
            ctx.font = '14px sans-serif';
            ctx.fillText(`G=${g}`, gX + 20, y + boxSize + 20);

            // + sign
            ctx.fillStyle = '#888';
            ctx.font = 'bold 24px sans-serif';
            ctx.fillText('+', gX + boxSize + 10, y + boxSize / 2 + 8);

            // B component
            const bX = gX + boxSize + spacing + 20;
            ctx.fillStyle = `rgb(0, 0, ${b})`;
            ctx.fillRect(bX, y, boxSize, boxSize);
            ctx.fillStyle = '#fff';
            ctx.font = '14px sans-serif';
            ctx.fillText(`B=${b}`, bX + 20, y + boxSize + 20);

            // = sign
            ctx.fillStyle = '#888';
            ctx.font = 'bold 24px sans-serif';
            ctx.fillText('=', bX + boxSize + 10, y + boxSize / 2 + 8);

            // Result
            const resX = bX + boxSize + spacing + 20;
            ctx.fillStyle = `rgb(${r}, ${g}, ${b})`;
            ctx.fillRect(resX, y, boxSize, boxSize);
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.strokeRect(resX, y, boxSize, boxSize);

            // Vector notation
            ctx.fillStyle = '#fff';
            ctx.font = '14px sans-serif';
            ctx.fillText(`(${r}, ${g}, ${b})`, resX + 10, y + boxSize + 20);

            // Title
            ctx.font = 'bold 14px sans-serif';
            ctx.fillText('颜色 = 3D向量！', 20, 25);
        }

        // Sliders
        ['color-r', 'color-g', 'color-b'].forEach((id, i) => {
            const slider = document.getElementById(id);
            if (slider) {
                slider.addEventListener('input', (e) => {
                    const val = parseInt(e.target.value);
                    if (i === 0) r = val;
                    else if (i === 1) g = val;
                    else b = val;
                    draw();
                });
            }
        });

        draw();
    })();
};
