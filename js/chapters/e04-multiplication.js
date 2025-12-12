/**
 * Chapter 4: Matrix Multiplication as Composition
 */
window.chapterScripts['e04-multiplication'] = function () {
    console.log('Initializing Chapter 4 (Matrix Multiplication)');

    const canvasEl = document.getElementById('viz-composition');
    if (!canvasEl) {
        console.warn('Canvas viz-composition not found');
        return;
    }

    const canvas = new CanvasSpace('viz-composition', { interactive: true });

    // Two preset transformations
    // M1 = Rotation 45°, M2 = Shear
    const M1 = {
        a: Math.cos(Math.PI / 4), b: -Math.sin(Math.PI / 4),
        c: Math.sin(Math.PI / 4), d: Math.cos(Math.PI / 4)
    };
    const M2 = { a: 1, b: 0.5, c: 0, d: 1 };

    // Composite M2 * M1
    function multiplyMatrices(A, B) {
        return {
            a: A.a * B.a + A.b * B.c,
            b: A.a * B.b + A.b * B.d,
            c: A.c * B.a + A.d * B.c,
            d: A.c * B.b + A.d * B.d
        };
    }

    const M_composite = multiplyMatrices(M2, M1);

    // Animation state
    let animState = 'identity'; // identity, after_m1, after_m2, composite
    let currentMatrix = { a: 1, b: 0, c: 0, d: 1 };

    // Draw function
    canvas.draw = function () {
        this.clear();
        const ctx = this.ctx;
        const origin = this.toCanvas(0, 0);

        const i_hat = { x: currentMatrix.a, y: currentMatrix.c };
        const j_hat = { x: currentMatrix.b, y: currentMatrix.d };

        // Draw grid
        ctx.save();
        ctx.strokeStyle = 'rgba(74, 158, 255, 0.15)';
        ctx.lineWidth = 1;

        const range = 10;
        for (let k = -range; k <= range; k++) {
            const s1 = this.toCanvas(k * i_hat.x - range * j_hat.x, k * i_hat.y - range * j_hat.y);
            const e1 = this.toCanvas(k * i_hat.x + range * j_hat.x, k * i_hat.y + range * j_hat.y);
            ctx.beginPath(); ctx.moveTo(s1.x, s1.y); ctx.lineTo(e1.x, e1.y); ctx.stroke();

            const s2 = this.toCanvas(-range * i_hat.x + k * j_hat.x, -range * i_hat.y + k * j_hat.y);
            const e2 = this.toCanvas(range * i_hat.x + k * j_hat.x, range * i_hat.y + k * j_hat.y);
            ctx.beginPath(); ctx.moveTo(s2.x, s2.y); ctx.lineTo(e2.x, e2.y); ctx.stroke();
        }
        ctx.restore();

        // Draw basis vectors
        const iEnd = this.toCanvas(i_hat.x, i_hat.y);
        const jEnd = this.toCanvas(j_hat.x, j_hat.y);
        this.drawArrow(origin, iEnd, '#f97316', 'î');
        this.drawArrow(origin, jEnd, '#10b981', 'ĵ');

        // State label
        ctx.fillStyle = '#fff';
        ctx.font = 'bold 14px sans-serif';
        let label = '';
        if (animState === 'identity') label = '初始状态 (Identity)';
        else if (animState === 'after_m1') label = '第一步: 旋转 45° (M₁)';
        else if (animState === 'after_m2') label = '第二步: 剪切 (M₂)';
        else if (animState === 'composite') label = '一步到位: M₂·M₁';
        ctx.fillText(label, 20, 30);
    };

    // Buttons
    document.getElementById('btn-step-by-step')?.addEventListener('click', () => {
        // Animate step by step
        animState = 'identity';
        currentMatrix = { a: 1, b: 0, c: 0, d: 1 };
        canvas.draw();

        setTimeout(() => {
            animState = 'after_m1';
            currentMatrix = M1;
            canvas.draw();
        }, 800);

        setTimeout(() => {
            animState = 'after_m2';
            currentMatrix = multiplyMatrices(M2, M1);
            canvas.draw();
        }, 1600);
    });

    document.getElementById('btn-composite')?.addEventListener('click', () => {
        animState = 'composite';
        currentMatrix = M_composite;
        canvas.draw();
    });

    document.getElementById('btn-reset-comp')?.addEventListener('click', () => {
        animState = 'identity';
        currentMatrix = { a: 1, b: 0, c: 0, d: 1 };
        canvas.draw();
    });

    // Initial draw
    canvas.draw();
};
