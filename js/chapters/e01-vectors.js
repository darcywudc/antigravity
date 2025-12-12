/**
 * Chapter 1: Vectors
 * Essence of Linear Algebra style
 */
window.chapterScripts['e01-vectors'] = function () {
    console.log('Initializing Chapter 1 (Essence)');

    const canvas = new CanvasSpace('viz-vector-basic', { interactive: true });

    // Initial vector state
    const vector = new Vector2D(2, 1);

    // Add draggable vector
    canvas.addVector(vector, {
        color: '#4a9eff',
        label: 'v',
        isDraggable: true,
        onDrag: (v) => {
            updateDisplay(v);
        }
    });

    // Update the math display text
    function updateDisplay(v) {
        const display = document.getElementById('vector-display');
        if (display && window.katex) {
            // Round to 1 decimal place for cleaner display
            const x = Math.round(v.x * 10) / 10;
            const y = Math.round(v.y * 10) / 10;

            katex.render(
                `\\vec{v} = \\begin{bmatrix} ${x} \\\\ ${y} \\end{bmatrix}`,
                display,
                { throwOnError: false, displayMode: true }
            );
        }
    }

    // Initial draw
    canvas.draw();
    updateDisplay(vector);
};
