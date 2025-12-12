/**
 * Chapter 2 Scripts: Matrices & Linear Transformations
 */
window.chapterScripts['ch2-matrices'] = function () {
    console.log('Initializing Chapter 2 (Loaded from external file)');

    const canvas = new CanvasSpace('viz-transform', { interactive: true });

    // 当前矩阵状态
    let matrix = new Matrix(2, 2, [[1, 0], [0, 1]]);

    // 输入框元素
    const inputs = [
        [document.getElementById('m00'), document.getElementById('m01')],
        [document.getElementById('m10'), document.getElementById('m11')]
    ];

    // 动画状态
    let targetMatrix = matrix.clone();
    let currentMatrix = matrix.clone();
    let isAnimating = false;

    // 扩展 CanvasSpace 以绘制变换后的网格
    canvas.drawTransformedGrid = function (m) {
        const ctx = this.ctx;
        const { width, height, origin, scale } = this.options;

        ctx.save();
        ctx.lineWidth = 1;
        ctx.strokeStyle = 'rgba(74, 158, 255, 0.3)'; // 变换后的网格颜色

        // 变换函数
        const transform = (x, y) => {
            // 应用矩阵变换
            const tx = m.get(0, 0) * x + m.get(0, 1) * y;
            const ty = m.get(1, 0) * x + m.get(1, 1) * y;
            return this.toCanvas(tx, ty);
        };

        ctx.beginPath();

        // 绘制变换后的垂直线
        for (let x = -20; x <= 20; x++) {
            const start = transform(x, -20);
            const end = transform(x, 20);
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
        }

        // 绘制变换后的水平线
        for (let y = -20; y <= 20; y++) {
            const start = transform(-20, y);
            const end = transform(20, y);
            ctx.moveTo(start.x, start.y);
            ctx.lineTo(end.x, end.y);
        }

        ctx.stroke();

        // 绘制基向量变换后的位置
        const i_hat = transform(1, 0);
        const j_hat = transform(0, 1);
        const originPt = this.toCanvas(0, 0);

        this.drawArrow(originPt, i_hat, '#f97316', 'i\'');
        this.drawArrow(originPt, j_hat, '#10b981', 'j\'');

        ctx.restore();
    };

    // 重写 draw 方法以包含变换网格
    const originalDraw = canvas.draw.bind(canvas);
    canvas.draw = function () {
        this.clear();
        this.drawGrid(); // 原始背景网格
        this.drawTransformedGrid(currentMatrix);
        // 不绘制普通向量，因为我们需要专门绘制变换基向量
    };

    // 动画循环
    const animate = () => {
        if (!isAnimating) return;

        let diff = 0;
        const speed = 0.1;

        for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 2; j++) {
                const current = currentMatrix.get(i, j);
                const target = targetMatrix.get(i, j);
                const delta = (target - current) * speed;

                if (Math.abs(target - current) > 0.001) {
                    currentMatrix.set(i, j, current + delta);
                    diff += Math.abs(delta);
                } else {
                    currentMatrix.set(i, j, target);
                }
            }
        }

        canvas.draw();

        if (diff > 0.001) {
            requestAnimationFrame(animate);
        } else {
            isAnimating = false;
        }
    };

    const updateMatrixFromInputs = () => {
        const m00 = parseFloat(inputs[0][0].value) || 0;
        const m01 = parseFloat(inputs[0][1].value) || 0;
        const m10 = parseFloat(inputs[1][0].value) || 0;
        const m11 = parseFloat(inputs[1][1].value) || 0;

        targetMatrix = new Matrix(2, 2, [[m00, m01], [m10, m11]]);
        if (!isAnimating) {
            isAnimating = true;
            animate();
        }
    };

    const setMatrixInputs = (m) => {
        inputs[0][0].value = m.get(0, 0);
        inputs[0][1].value = m.get(0, 1);
        inputs[1][0].value = m.get(1, 0);
        inputs[1][1].value = m.get(1, 1);
        updateMatrixFromInputs();
    };

    // 绑定输入框事件
    inputs.flat().forEach(input => {
        if (input) input.addEventListener('input', updateMatrixFromInputs);
    });

    // 按钮事件
    const bindBtn = (id, callback) => {
        const btn = document.getElementById(id);
        if (btn) btn.addEventListener('click', callback);
    };

    bindBtn('btn-identity', () => {
        setMatrixInputs(Matrix.identity(2));
    });

    bindBtn('btn-rotate', () => {
        const angle = Math.PI / 4; // 45度
        const cos = Number(Math.cos(angle).toFixed(2));
        const sin = Number(Math.sin(angle).toFixed(2));
        setMatrixInputs(new Matrix(2, 2, [[cos, -sin], [sin, cos]]));
    });

    bindBtn('btn-shear', () => {
        setMatrixInputs(new Matrix(2, 2, [[1, 1], [0, 1]]));
    });

    bindBtn('btn-scale', () => {
        setMatrixInputs(new Matrix(2, 2, [[1.5, 0], [0, 0.5]]));
    });

    // 初始绘制
    canvas.draw();
};
