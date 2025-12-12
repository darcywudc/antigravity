/**
 * Canvas2D 可视化引擎
 */

class CanvasSpace {
    constructor(canvasId, options = {}) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) {
            console.error(`[CanvasSpace] Error: element '${canvasId}' not found`);
            window.logDebug && window.logDebug(`CanvasSpace: element '${canvasId}' not found`, 'error');
            return;
        }

        this.ctx = this.canvas.getContext('2d');
        this.width = this.canvas.width = this.canvas.offsetWidth;
        this.height = this.canvas.height = this.canvas.offsetHeight;

        // 配置选项
        this.options = {
            gridSize: 40,
            gridColor: '#1a1a25',
            axisColor: '#ffffff',
            axisWidth: 2,
            subGridColor: '#12121a',
            bgColor: '#0d0d12',
            origin: null, // Default to center if null
            scale: 40,
            interactive: true,
            ...options
        };

        // 如果没有指定 origin，默认居中
        if (!this.options.origin) {
            this.options.origin = { x: this.width / 2, y: this.height / 2 };
        }

        // 状态
        this.vectors = [];
        this.animating = false;
        this.draggables = [];
        this.draggedItem = null;
        this.mousePos = { x: 0, y: 0 };

        this.init();
        window.logDebug && window.logDebug(`CanvasSpace initialized: ${canvasId} (${this.width}x${this.height})`);
    }

    init() {
        this.resizeObserver = new ResizeObserver(() => this.resize());
        this.resizeObserver.observe(this.canvas);

        if (this.options.interactive) {
            this.setupInteraction();
        }

        this.draw();
    }

    resize() {
        if (!this.canvas) return;
        this.width = this.canvas.width = this.canvas.offsetWidth;
        this.height = this.canvas.height = this.canvas.offsetHeight;

        // 如果没有自定义 Origin，则保持居中
        // 简单的判断：如果当前 origin 正好是旧中心，或者我们添加一个标志
        // 这里为了修复 Ch3 的问题（它依赖固定坐标），我们暂时不重置 Origin
        // 除非我们需要自适应。
        // 更好的做法：在 Ch3 里重写 resize 或者在 options 里加 autoCenter: true
        if (this.options.autoCenter) {
            this.options.origin = { x: this.width / 2, y: this.height / 2 };
        }

        this.draw();
    }

    // ... (rest of methods)

    toCanvas(x, y) {
        if (typeof x === 'object') { y = x.y; x = x.x; }
        // Add check
        if (!this.options.origin) return { x: 0, y: 0 };
        return {
            x: this.options.origin.x + x * this.options.scale,
            y: this.options.origin.y - y * this.options.scale
        };
    }

    toMath(x, y) {
        if (!this.options.origin) return { x: 0, y: 0 };
        return {
            x: (x - this.options.origin.x) / this.options.scale,
            y: (this.options.origin.y - y) / this.options.scale
        };
    }

    setupInteraction() {
        // ... existing code ...
        this.canvas.addEventListener('mousedown', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;

            for (let item of this.draggables) {
                const pos = this.toCanvas(item.v.x, item.v.y);
                const dist = Math.hypot(x - pos.x, y - pos.y);

                if (dist < 10) {
                    this.draggedItem = item;
                    this.canvas.style.cursor = 'grabbing';
                    return;
                }
            }
        });

        this.canvas.addEventListener('mousemove', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            this.mousePos = { x, y };

            if (this.draggedItem) {
                const mathPos = this.toMath(x, y);
                if (e.shiftKey) {
                    mathPos.x = Math.round(mathPos.x);
                    mathPos.y = Math.round(mathPos.y);
                }

                this.draggedItem.v.x = mathPos.x;
                this.draggedItem.v.y = mathPos.y;

                if (this.draggedItem.onUpdate) {
                    this.draggedItem.onUpdate(this.draggedItem.v);
                }

                this.draw();
            } else {
                let hovering = false;
                for (let item of this.draggables) {
                    const pos = this.toCanvas(item.v.x, item.v.y);
                    const dist = Math.hypot(x - pos.x, y - pos.y);
                    if (dist < 10) {
                        hovering = true;
                        break;
                    }
                }
                this.canvas.style.cursor = hovering ? 'grab' : 'crosshair';
            }
        });

        window.addEventListener('mouseup', () => {
            this.draggedItem = null;
            this.canvas.style.cursor = 'crosshair';
        });
    }

    addVector(vector, color = '#4a9eff', label = null) {
        if (!vector) return;
        const obj = { v: vector, color, label };
        this.vectors.push(obj);
        return obj;
    }

    makeDraggable(vectorObj, onUpdate) {
        if (!vectorObj) return;
        vectorObj.draggable = true;
        vectorObj.onUpdate = onUpdate;
        this.draggables.push(vectorObj);
    }

    clear() {
        this.ctx.fillStyle = this.options.bgColor;
        this.ctx.fillRect(0, 0, this.width, this.height);
    }

    drawGrid() {
        const ctx = this.ctx;
        const width = this.width;
        const height = this.height;
        const origin = this.options.origin;
        const scale = this.options.scale;
        if (!origin) return;

        ctx.lineWidth = 1;
        ctx.strokeStyle = '#1d1d2b';
        ctx.beginPath();

        for (let x = origin.x % scale; x < width; x += scale) {
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height);
        }

        for (let y = origin.y % scale; y < height; y += scale) {
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
        }

        ctx.stroke();

        ctx.strokeStyle = this.options.axisColor;
        ctx.lineWidth = 2;
        ctx.beginPath();

        ctx.moveTo(0, origin.y);
        ctx.lineTo(width, origin.y);

        ctx.moveTo(origin.x, 0);
        ctx.lineTo(origin.x, height);

        ctx.stroke();

        ctx.fillStyle = '#666';
        ctx.font = '10px monospace';
        ctx.textAlign = 'center';

        for (let x = origin.x % scale; x < width; x += scale) {
            if (Math.abs(x - origin.x) < 1) continue;
            const val = Math.round((x - origin.x) / scale);
            ctx.fillText(val, x, origin.y + 15);
        }

        ctx.textAlign = 'right';
        for (let y = origin.y % scale; y < height; y += scale) {
            if (Math.abs(y - origin.y) < 1) continue;
            const val = Math.round((origin.y - y) / scale);
            ctx.fillText(val, origin.x - 5, y + 4);
        }

        ctx.fillText('O', origin.x - 5, origin.y + 15);
    }

    drawArrow(from, to, color, label = null) {
        const ctx = this.ctx;
        const headLen = 10;
        const angle = Math.atan2(to.y - from.y, to.x - from.x);

        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = 2;

        ctx.beginPath();
        ctx.moveTo(from.x, from.y);
        ctx.lineTo(to.x, to.y);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(to.x, to.y);
        ctx.lineTo(to.x - headLen * Math.cos(angle - Math.PI / 6), to.y - headLen * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(to.x - headLen * Math.cos(angle + Math.PI / 6), to.y - headLen * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fill();

        if (label) {
            ctx.fillStyle = color;
            ctx.font = 'bold 14px sans-serif';
            ctx.textAlign = 'center';
            ctx.fillText(label, to.x + 15, to.y - 15);

            const mathVec = this.toMath(to.x, to.y);
            ctx.font = '10px monospace';
            ctx.fillText(`(${mathVec.x.toFixed(1)}, ${mathVec.y.toFixed(1)})`, to.x + 15, to.y);
        }
    }

    draw() {
        this.clear();
        this.drawGrid();

        for (let item of this.vectors) {
            const start = item.origin ? this.toCanvas(item.origin) : this.options.origin;
            const end = this.toCanvas(item.v.x, item.v.y);

            let finalEnd = end;
            if (item.origin) {
                const startMath = item.origin;
                const endMath = { x: startMath.x + item.v.x, y: startMath.y + item.v.y };
                finalEnd = this.toCanvas(endMath);
            }

            this.drawArrow(start, finalEnd, item.color, item.label);
        }
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { CanvasSpace };
}
