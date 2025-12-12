/**
 * 应用主逻辑
 */

// Debug Logger
window.logDebug = function (msg, type = 'info') {
    const consoleDiv = document.getElementById('debug-console');
    if (consoleDiv) {
        const line = document.createElement('div');
        line.textContent = `> ${msg}`;
        if (type === 'error') line.className = 'log-error';
        else line.className = 'log-info';
        consoleDiv.appendChild(line);
        consoleDiv.scrollTop = consoleDiv.scrollHeight;
    }
    console.log(`[${type.toUpperCase()}] ${msg}`);
};

window.onerror = function (msg, url, line) {
    window.logDebug(`${msg} (${url}:${line})`, 'error');
    return false;
};

const App = {
    currentChapter: null,

    init() {
        window.logDebug('App initializing...');
        this.bindNavigation();
        this.loadInitialChapter();
    },

    bindNavigation() {
        const links = document.querySelectorAll('.nav-link');
        links.forEach(link => {
            link.addEventListener('click', (e) => {
                e.preventDefault();
                const chapterId = link.getAttribute('data-chapter');
                if (chapterId) {
                    this.loadChapter(chapterId);
                    links.forEach(l => l.classList.remove('active'));
                    link.classList.add('active');
                }
            });
        });
    },

    loadInitialChapter() {
        const hash = window.location.hash.slice(1);
        window.logDebug(`Initial hash: ${hash}`);
        let chapterId = 'e01-vectors';

        // Map short hash to ID
        const map = {
            'ch1': 'e01-vectors',
            'ch2': 'e02-combinations',
            'ch3': 'e03-transformations',
            'ch4': 'e04-multiplication',
            'ch5': 'e05-3d-transform',
            'ch6': 'e06-determinant',
            'ch7': 'e07-inverse',
            'ch8': 'e08-nonsquare',
            'ch9': 'e09-dot-product',
            'ch10': 'e10-cross-product',
            'ch11': 'e11-cross-transform',
            'ch12': 'e12-cramer',
            'ch13': 'e13-change-basis',
            'ch14': 'e14-eigenvectors',
            'ch15': 'e15-abstract',
            // Also support direct e0X style hashes
            'e01': 'e01-vectors',
            'e02': 'e02-combinations',
            'e03': 'e03-transformations',
            'e04': 'e04-multiplication',
            'e05': 'e05-3d-transform',
            'e06': 'e06-determinant',
            'e07': 'e07-inverse',
            'e08': 'e08-nonsquare',
            'e09': 'e09-dot-product',
            'e10': 'e10-cross-product',
            'e11': 'e11-cross-transform',
            'e12': 'e12-cramer',
            'e13': 'e13-change-basis',
            'e14': 'e14-eigenvectors',
            'e15': 'e15-abstract'
        };
        if (map[hash]) chapterId = map[hash];

        this.loadChapter(chapterId);
    },

    async loadChapter(chapterId) {
        window.logDebug(`Loading chapter: ${chapterId}`);
        const main = document.getElementById('main-content');

        try {
            const response = await fetch(`chapters/${chapterId}.html`);
            if (!response.ok) throw new Error(`HTTP ${response.status}`);

            const html = await response.text();
            main.innerHTML = html;
            window.logDebug('HTML content injected');

            // Wait a tick for DOM layout
            setTimeout(() => {
                // Initialize KaTeX
                if (window.renderMathInElement) {
                    try {
                        renderMathInElement(main, {
                            delimiters: [
                                { left: '$$', right: '$$', display: true },
                                { left: '$', right: '$', display: false }
                            ],
                            throwOnError: false
                        });
                        window.logDebug('Math rendered');
                    } catch (e) {
                        window.logDebug(`Math render error: ${e.message}`, 'error');
                    }
                } else {
                    window.logDebug('KaTeX not loaded yet', 'error');
                }

                // Load Script
                this.loadAndRunScript(chapterId);

            }, 50);

            const shortId = chapterId.split('-')[0];
            history.pushState(null, null, `#${shortId}`);

        } catch (error) {
            window.logDebug(`Load failed: ${error.message}`, 'error');
            main.innerHTML = `<div class="error">加载章节失败: ${error.message}</div>`;
        }
    },

    loadAndRunScript(chapterId) {
        const oldScript = document.getElementById('chapter-script');
        if (oldScript) oldScript.remove();

        const scriptUrl = `js/chapters/${chapterId}.js`;
        window.logDebug(`Fetching script: ${scriptUrl}`);

        const script = document.createElement('script');
        script.id = 'chapter-script';
        script.src = scriptUrl;

        script.onload = () => {
            window.logDebug(`Script loaded: ${chapterId}`);
            if (window.chapterScripts && window.chapterScripts[chapterId]) {
                try {
                    window.chapterScripts[chapterId]();
                    window.logDebug(`Chapter ${chapterId} initialized successfully`);
                } catch (e) {
                    window.logDebug(`Error executing chapter script: ${e.message}`, 'error');
                    console.error(e);
                }
            } else {
                window.logDebug(`No init function found for ${chapterId}`, 'error');
            }
        };

        script.onerror = (e) => {
            window.logDebug(`Failed to load script: ${scriptUrl}`, 'error');
        };

        document.body.appendChild(script);
    }
};

window.chapterScripts = {};

document.addEventListener('DOMContentLoaded', () => {
    try {
        App.init();
    } catch (e) {
        window.logDebug(`App init error: ${e.message}`, 'error');
    }
});
