/**
 * Chapter 1 Scripts: Vectors & Vector Spaces
 */
window.chapterScripts['ch1-vectors'] = function () {
    console.log('Initializing Chapter 1 (Loaded from external file)');

    // 确保 Canvas 元素存在
    if (!document.getElementById('viz-vector-basic')) {
        console.error('Canvas viz-vector-basic not found');
        return;
    }

    // --- 演示 1: 基础向量 ---
    const canvas1 = new CanvasSpace('viz-vector-basic', { interactive: true });

    // 初始向量
    const v1 = new Vector2D(2, 1);
    const vecObj1 = canvas1.addVector(v1, '#4a9eff', 'v');

    // 设为可拖拽
    canvas1.makeDraggable(vecObj1, (newV) => {
        // 更新显示的数值
        const display = document.getElementById('vector-display');
        katex.render(`\\mathbf{v} = \\begin{bmatrix} ${newV.x.toFixed(1)} \\\\ ${newV.y.toFixed(1)} \\end{bmatrix}`, display, {
            displayMode: true
        });
    });


    // --- 演示 2: 线性组合 ---
    const canvas2 = new CanvasSpace('viz-linear-combo', { interactive: true });

    // 基向量 u 和 v
    const u = new Vector2D(2, 0);
    const v = new Vector2D(1, 2);

    // 系数
    let c1 = 1.0;
    let c2 = 1.0;

    // 添加向量对象
    // u (红)
    const uObj = canvas2.addVector(u, '#f97316', 'u'); // Orange
    // v (绿)
    const vObj = canvas2.addVector(v, '#10b981', 'v'); // Green

    // w (结果, 黄) - 初始值
    const w = u.scale(c1).add(v.scale(c2));
    const wObj = canvas2.addVector(w, '#fbbf24', 'w'); // Yellow

    // 更新函数
    const updateCombo = () => {
        // 计算新的线性组合
        const scaledU = u.scale(c1);
        const scaledV = v.scale(c2);
        const result = scaledU.add(scaledV);

        // 更新 w 的坐标
        w.x = result.x;
        w.y = result.y;

        // 更新 Slider 显示数值
        document.getElementById('val-c1').textContent = c1.toFixed(1);
        document.getElementById('val-c2').textContent = c2.toFixed(1);

        // 更新公式
        const display = document.getElementById('combo-display');
        katex.render(`\\mathbf{w} = ${c1.toFixed(1)}\\mathbf{u} + ${c2.toFixed(1)}\\mathbf{v} = \\begin{bmatrix} ${result.x.toFixed(1)} \\\\ ${result.y.toFixed(1)} \\end{bmatrix}`, display, {
            displayMode: true
        });

        canvas2.draw();
    };

    // 允许拖拽基向量 u, v
    canvas2.makeDraggable(uObj, () => updateCombo());
    canvas2.makeDraggable(vObj, () => updateCombo());

    // 绑定滑块事件
    const slider1 = document.getElementById('slider-c1');
    const slider2 = document.getElementById('slider-c2');

    if (slider1 && slider2) {
        slider1.addEventListener('input', (e) => {
            c1 = parseFloat(e.target.value);
            updateCombo();
        });

        slider2.addEventListener('input', (e) => {
            c2 = parseFloat(e.target.value);
            updateCombo();
        });
    }

    // 初始更新
    updateCombo();
};
