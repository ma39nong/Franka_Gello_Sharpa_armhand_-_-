"""NiceGUI styles for the Linker Hand control page."""

from __future__ import annotations

from typing import Any


def install_styles(ui: Any) -> None:
    """Install the fresh, natural control-studio theme."""

    ui.add_head_html(
        """
        <link rel="preconnect" href="https://fonts.googleapis.com">
        <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
        <link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=Instrument+Sans:wght@400;500;600;700&display=swap" rel="stylesheet">
        <style>
        :root {
            --hand-bg: #f5f8f3;
            --hand-bg-band: #eaf3ec;
            --hand-surface: #ffffff;
            --hand-surface-soft: #f8fbf7;
            --hand-surface-green: #edf7ef;
            --hand-surface-blue: #edf6f8;
            --hand-ink: #1f2a25;
            --hand-muted: #637168;
            --hand-faint: #8b9a90;
            --hand-line: #d8e2d9;
            --hand-line-strong: #bfd0c2;
            --hand-primary: #2f8f5b;
            --hand-primary-strong: #176e4a;
            --hand-primary-soft: rgba(47, 143, 91, 0.12);
            --hand-blue: #3b7ea1;
            --hand-blue-soft: rgba(59, 126, 161, 0.12);
            --hand-amber: #c98d27;
            --hand-danger: #c64c45;
            --hand-danger-soft: rgba(198, 76, 69, 0.12);
        }

        html, body, #app, .nicegui-layout, .q-page-container, .q-page,
        .nicegui-content {
            min-height: 100%;
        }

        .nicegui-content {
            padding: 0 !important;
        }

        body {
            color: var(--hand-ink);
            background: var(--hand-bg);
            font-family: "Instrument Sans", sans-serif;
            overflow-x: hidden;
        }

        body::-webkit-scrollbar, .preset-scroll::-webkit-scrollbar,
        .hands-grid::-webkit-scrollbar, .finger-grid::-webkit-scrollbar,
        .tactile-grid::-webkit-scrollbar {
            width: 8px;
            height: 8px;
        }

        body::-webkit-scrollbar-track, .preset-scroll::-webkit-scrollbar-track,
        .hands-grid::-webkit-scrollbar-track, .finger-grid::-webkit-scrollbar-track,
        .tactile-grid::-webkit-scrollbar-track {
            background: rgba(31, 42, 37, 0.06);
        }

        body::-webkit-scrollbar-thumb, .preset-scroll::-webkit-scrollbar-thumb,
        .hands-grid::-webkit-scrollbar-thumb, .finger-grid::-webkit-scrollbar-thumb,
        .tactile-grid::-webkit-scrollbar-thumb {
            background: rgba(47, 143, 91, 0.34);
            border-radius: 8px;
        }

        .hand-shell {
            min-height: 100vh;
            box-sizing: border-box;
            color: var(--hand-ink);
            gap: 0 !important;
        }

        .top-appbar {
            position: sticky;
            top: 0;
            z-index: 40;
            box-sizing: border-box;
            min-height: 72px;
            padding: 12px 28px;
            border-bottom: 1px solid rgba(191, 208, 194, 0.78);
            background: var(--hand-bg);
        }

        .brand-title {
            color: var(--hand-ink);
            font-size: 23px;
            font-weight: 700;
            letter-spacing: 0;
            white-space: nowrap;
        }

        .header-trajectory {
            min-width: 0;
            flex: 1;
            justify-content: flex-end;
        }

        .trajectory-field {
            width: min(38vw, 480px);
            min-width: 240px;
        }

        .control-canvas {
            box-sizing: border-box;
            flex: 1;
            min-height: 0;
            padding: 24px;
            padding-bottom: 108px;
        }

        .model-badge {
            color: var(--hand-primary-strong);
            background: var(--hand-surface-green);
            border: 1px solid rgba(47, 143, 91, 0.20);
            border-radius: 999px;
            padding: 6px 11px;
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 600;
            letter-spacing: 0;
            white-space: nowrap;
        }

        .frame-badge {
            color: var(--hand-blue);
            background: var(--hand-surface-blue);
            border-color: rgba(59, 126, 161, 0.22);
        }

        .soft-button, .emergency-button, .system-lock-button {
            border-radius: 8px !important;
            min-height: 36px;
            font-size: 13px;
            font-weight: 600;
            letter-spacing: 0;
            text-transform: none;
        }

        .soft-button {
            color: var(--hand-primary-strong) !important;
            background: rgba(255, 255, 255, 0.54) !important;
        }

        .emergency-button {
            color: #ffffff !important;
            background: var(--hand-danger) !important;
        }

        .system-lock-button {
            color: var(--hand-danger) !important;
            border: 1px solid rgba(198, 76, 69, 0.42);
        }

        .icon-button {
            width: 34px;
            height: 34px;
            color: var(--hand-muted);
            border-radius: 999px !important;
            background: rgba(255, 255, 255, 0.62) !important;
        }

        .icon-button:hover {
            color: var(--hand-primary-strong) !important;
            background: var(--hand-primary-soft) !important;
        }

        .hands-grid {
            min-height: 0;
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 18px;
            align-items: start;
            overflow: visible;
        }

        .hand-panel {
            min-width: 0;
            min-height: 0;
            display: flex;
            flex-direction: column;
            gap: 14px !important;
            padding: 16px !important;
            border: 1px solid rgba(191, 208, 194, 0.82);
            border-radius: 8px !important;
            background: rgba(255, 255, 255, 0.86);
        }

        .hand-card-header {
            display: grid;
            grid-template-columns: minmax(190px, 1fr) minmax(132px, 160px) minmax(96px, 112px);
            align-items: end;
            gap: 14px;
            padding: 4px 4px 14px;
            border-bottom: 1px solid var(--hand-line);
        }

        .hand-identity, .hand-model-control, .hand-emergency-zone {
            min-width: 0;
        }

        .hand-title {
            color: var(--hand-ink);
            font-size: 24px;
            font-weight: 700;
            letter-spacing: 0;
        }

        .hand-icon {
            color: var(--hand-primary);
            font-size: 30px;
        }

        .hand-subtitle {
            color: var(--hand-muted);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 500;
            letter-spacing: 0;
            margin-top: 2px;
        }

        .hand-field-label, .section-title {
            color: var(--hand-muted);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 600;
            letter-spacing: 0;
        }

        .section-note {
            color: var(--hand-faint);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 500;
            white-space: nowrap;
        }

        .runtime-inline {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 10px;
        }

        .runtime-control-card {
            min-width: 0;
            padding: 12px !important;
            border: 1px solid rgba(191, 208, 194, 0.80);
            border-radius: 8px !important;
            background: var(--hand-surface-green);
            display: flex;
            flex-direction: column;
        }

        .runtime-icon {
            color: var(--hand-primary);
            font-size: 18px;
        }

        .runtime-value {
            color: var(--hand-primary-strong);
            font-family: "IBM Plex Mono", monospace;
            font-size: 13px;
            font-weight: 600;
            min-width: 36px;
            text-align: right;
        }

        .joint-surface {
            display: flex;
            flex-direction: column;
            min-width: 0;
            padding: 14px;
            border: 1px solid rgba(191, 208, 194, 0.82);
            border-radius: 8px;
            background: var(--hand-surface-soft);
        }

        .finger-grid {
            min-height: 0;
            width: 100%;
            overflow-x: auto;
            overflow-y: visible;
            display: grid;
            grid-template-columns: repeat(5, minmax(156px, 1fr));
            gap: 8px;
        }

        .finger-card {
            min-width: 156px;
            min-height: 0;
            padding: 12px !important;
            overflow: hidden;
            border: 1px solid rgba(216, 226, 217, 0.90);
            border-radius: 8px !important;
            background: #ffffff;
            box-shadow: none;
            display: flex;
            flex-direction: column;
        }

        .finger-name {
            color: var(--hand-primary-strong);
            padding-bottom: 8px;
            border-bottom: 1px solid var(--hand-line);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 600;
            letter-spacing: 0;
            text-align: left;
        }

        .joint-row {
            padding: 0;
        }

        .joint-title {
            color: var(--hand-muted);
            font-family: "IBM Plex Mono", monospace;
            font-size: 11px;
            font-weight: 500;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }

        .joint-value {
            color: var(--hand-ink);
            background: var(--hand-surface-green);
            border: 1px solid rgba(47, 143, 91, 0.16);
            border-radius: 999px;
            padding: 1px 7px;
            font-family: "IBM Plex Mono", monospace;
            font-size: 11px;
            font-weight: 600;
            min-width: 34px;
            text-align: center;
        }

        .joint-control-row {
            min-height: 28px;
            min-width: 90px;
        }

        .joint-step-button {
            width: 26px;
            min-width: 26px !important;
            height: 26px;
            min-height: 26px !important;
            padding: 0 !important;
            border: 1px solid rgba(191, 208, 194, 0.92);
            color: var(--hand-muted) !important;
            background: #ffffff !important;
        }

        .joint-step-button:hover {
            color: var(--hand-primary-strong) !important;
            border-color: rgba(47, 143, 91, 0.40);
            background: var(--hand-primary-soft) !important;
        }

        .compact-slider {
            min-width: 34px;
            margin-top: -3px;
            margin-bottom: -6px;
        }

        .q-slider {
            min-height: 24px !important;
        }

        .q-slider__track {
            height: 3px;
            border-radius: 999px;
            background: rgba(99, 113, 104, 0.22);
        }

        .q-slider__selection {
            background: var(--hand-primary);
        }

        .q-slider__thumb {
            color: #ffffff;
        }

        .action-surface {
            display: grid;
            grid-template-columns: minmax(0, 0.92fr) minmax(0, 1.08fr);
            gap: 12px;
            align-items: stretch;
        }

        .preset-card, .tactile-card {
            min-width: 0;
            min-height: 220px;
            overflow: hidden;
            padding: 14px !important;
            border: 1px solid rgba(191, 208, 194, 0.78);
            border-radius: 8px !important;
            background: rgba(255, 255, 255, 0.70);
            box-shadow: none;
            display: flex;
            flex-direction: column;
        }

        .preset-scroll {
            max-height: 328px;
            overflow: auto;
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(148px, 1fr));
            gap: 8px;
            align-items: stretch;
        }

        .preset-button-group {
            min-width: 0;
            width: 100%;
            height: 40px;
            display: grid;
            grid-template-columns: minmax(0, 1fr) 36px;
            align-items: stretch;
            overflow: hidden;
            border: 1px solid var(--hand-line-strong);
            border-radius: 8px;
            background: #ffffff;
            transition:
                border-color 140ms ease,
                background 140ms ease;
        }

        .preset-button-group:hover {
            border-color: rgba(47, 143, 91, 0.42);
            background: #fbfdfb;
        }

        .preset-apply-button, .preset-remove-button {
            width: 100%;
            min-height: 38px !important;
            height: 38px;
            border-radius: 0 !important;
            background: transparent !important;
            color: var(--hand-ink) !important;
            font-size: 13px;
            font-weight: 600;
            letter-spacing: 0;
            text-transform: none;
        }

        .preset-apply-button {
            padding: 0 11px !important;
        }

        .preset-apply-button .q-btn__content {
            min-width: 0;
            width: 100%;
            justify-content: flex-start;
            overflow: hidden;
            white-space: nowrap;
            text-overflow: ellipsis;
        }

        .preset-remove-button {
            padding: 0 !important;
            border-left: 1px solid var(--hand-line);
            color: var(--hand-faint) !important;
        }

        .preset-remove-button:hover {
            color: var(--hand-danger) !important;
            background: var(--hand-danger-soft) !important;
        }

        .action-item {
            min-width: 0;
            width: 100%;
            display: grid !important;
            grid-template-columns: minmax(0, 1fr) auto;
        }

        .action-chip {
            width: 100%;
        }

        .action-chip .q-btn__content, .sequence-run-button .q-btn__content {
            min-width: 0;
            white-space: normal;
            line-height: 1.2;
        }

        .action-chip, .sequence-run-button {
            min-height: 40px;
        }

        .sequence-chip {
            grid-column: span 2;
            grid-template-columns: minmax(0, 1fr) auto auto auto;
        }

        .sequence-run-button {
            min-width: 0;
            width: 100%;
            color: var(--hand-blue) !important;
        }

        .mini-icon-button, .action-remove-button {
            width: 30px;
            min-width: 30px !important;
            height: 30px;
            min-height: 30px !important;
            border-radius: 999px !important;
        }

        .action-remove-button {
            color: var(--hand-faint) !important;
        }

        .action-remove-button:hover {
            color: var(--hand-danger) !important;
            background: var(--hand-danger-soft) !important;
        }

        .add-action-button {
            min-height: 40px;
            border-style: dashed !important;
            color: var(--hand-primary-strong) !important;
            background: var(--hand-primary-soft) !important;
        }

        .task-picker {
            width: min(640px, calc(100vw - 32px));
            border: 1px solid var(--hand-line);
            border-radius: 8px !important;
            background: var(--hand-surface);
        }

        .picker-grid {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
            gap: 8px;
        }

        .tactile-summary, .tactile-finger-value {
            color: var(--hand-blue);
            font-family: "IBM Plex Mono", monospace;
            font-size: 11px;
            font-weight: 600;
            letter-spacing: 0;
            white-space: nowrap;
        }

        .tactile-grid {
            min-width: 0;
            display: grid;
            grid-template-columns: repeat(5, minmax(78px, 1fr));
            gap: 7px;
            overflow-x: auto;
        }

        .tactile-finger {
            min-width: 78px;
            padding: 8px;
            border: 1px solid rgba(216, 226, 217, 0.92);
            border-radius: 8px;
            background: var(--hand-surface-blue);
        }

        .tactile-finger-name {
            color: var(--hand-ink);
            font-family: "IBM Plex Mono", monospace;
            font-size: 10px;
            font-weight: 600;
            letter-spacing: 0;
        }

        .tactile-matrix {
            display: grid;
            grid-template-columns: repeat(6, minmax(8px, 1fr));
            gap: 2px;
            margin-top: 8px;
        }

        .tactile-cell {
            aspect-ratio: 1 / 1;
            min-width: 8px;
            border: 1px solid rgba(255, 255, 255, 0.70);
            border-radius: 3px;
            background: rgba(59, 126, 161, 0.10);
        }

        .tactile-empty {
            min-height: 86px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            border: 1px dashed rgba(59, 126, 161, 0.25);
            border-radius: 8px;
            background: var(--hand-surface-blue);
        }

        .setup-drawer {
            border: 1px solid rgba(191, 208, 194, 0.78);
            border-radius: 8px;
            background: rgba(255, 255, 255, 0.66);
            overflow: hidden;
        }

        .setup-drawer .q-expansion-item__container {
            background: transparent;
        }

        .setup-drawer .q-item {
            min-height: 40px;
            color: var(--hand-muted);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 600;
            letter-spacing: 0;
        }

        .setup-drawer .q-expansion-item__content {
            padding: 0 12px 12px;
        }

        .setup-grid {
            display: grid;
            grid-template-columns: repeat(3, minmax(0, 1fr));
            align-items: stretch;
            gap: 8px;
            margin-top: 6px;
        }

        .library-action-group {
            min-width: 0;
            display: grid;
            grid-template-columns: minmax(0, 1fr) auto;
            align-items: stretch;
            gap: 8px;
        }

        .library-action-single {
            grid-template-columns: minmax(0, 1fr);
        }

        .task-input {
            min-width: 0;
            width: 100%;
        }

        .task-button {
            height: 40px;
            min-height: 40px !important;
            width: 128px;
            white-space: nowrap;
        }

        .task-button .q-btn__content {
            flex-wrap: nowrap;
            min-width: 0;
            white-space: nowrap;
        }

        .library-action-single .task-button {
            width: 100%;
        }

        .setup-grid .q-field {
            height: 40px;
        }

        .setup-grid .q-field__control {
            height: 40px;
            min-height: 40px !important;
        }

        .setup-grid .q-field__marginal {
            height: 40px;
        }

        .setup-grid .q-field__native,
        .setup-grid .q-field__input {
            min-height: 40px;
        }

        .command-console {
            position: fixed;
            left: 20px;
            bottom: 20px;
            z-index: 80;
            width: min(520px, calc(100vw - 40px));
            margin-top: 0;
            border: 1px solid rgba(191, 208, 194, 0.90);
            border-radius: 8px !important;
            background: #ffffff;
        }

        .command-details .q-expansion-item__container {
            background: transparent;
        }

        .command-count {
            color: var(--hand-blue);
            background: var(--hand-surface-blue);
            border: 1px solid rgba(59, 126, 161, 0.18);
            border-radius: 999px;
            padding: 3px 9px;
            font-family: "IBM Plex Mono", monospace;
            font-size: 11px;
            font-weight: 600;
            letter-spacing: 0;
            white-space: nowrap;
        }

        .command-log textarea {
            min-height: 150px !important;
            max-height: 42vh !important;
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            line-height: 1.4;
            color: var(--hand-muted) !important;
        }

        .status-bar {
            color: var(--hand-primary-strong);
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            font-weight: 600;
            letter-spacing: 0;
            min-width: 0;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }

        .q-card {
            color: var(--hand-ink);
        }

        .q-field__native, .q-field__input, .q-field__label,
        .q-select__dropdown-icon {
            color: var(--hand-ink) !important;
        }

        .q-field--outlined .q-field__control {
            background: rgba(255, 255, 255, 0.78);
            border-radius: 8px;
        }

        .q-field--outlined .q-field__control:before {
            border-color: rgba(191, 208, 194, 0.96) !important;
        }

        .q-field--outlined.q-field--focused .q-field__control:after {
            border-color: var(--hand-primary) !important;
        }

        .q-field__label {
            color: var(--hand-muted) !important;
            font-family: "IBM Plex Mono", monospace;
            font-size: 12px;
            letter-spacing: 0;
        }

        .q-menu {
            background: var(--hand-surface);
            color: var(--hand-ink);
            border: 1px solid var(--hand-line);
        }

        .q-tab {
            min-height: 34px;
            border-radius: 8px;
            color: var(--hand-muted);
            font-family: "IBM Plex Mono", monospace;
            font-size: 11px;
            font-weight: 600;
            letter-spacing: 0;
        }

        .q-tab--active {
            color: var(--hand-primary-strong);
            background: var(--hand-primary-soft);
        }

        .q-tabs__indicator {
            color: transparent;
        }

        .q-tab-panel {
            padding: 8px 0 0 !important;
        }

        .q-tabs__content {
            min-height: 34px;
            gap: 6px;
        }

        .q-btn {
            box-shadow: none !important;
        }

        .q-btn.bg-teal, .q-btn.text-teal,
        .q-btn.bg-cyan, .q-btn.text-cyan {
            color: var(--hand-primary-strong) !important;
        }

        .q-btn.bg-purple, .q-btn.text-purple {
            color: var(--hand-blue) !important;
        }

        .q-btn.bg-red, .q-btn.text-red {
            color: var(--hand-danger) !important;
        }

        .q-btn--outline:before {
            border-color: rgba(191, 208, 194, 0.92);
        }

        .q-btn:hover {
            color: var(--hand-primary-strong) !important;
        }

        .preset-remove-button:hover {
            color: var(--hand-danger) !important;
        }

        .q-notification {
            font-family: "Instrument Sans", sans-serif;
        }

        @media (max-width: 1180px) {
            .top-appbar {
                padding-left: 16px;
                padding-right: 16px;
            }

            .control-canvas {
                padding-left: 16px !important;
                padding-right: 16px !important;
            }

            .hands-grid, .action-surface {
                grid-template-columns: 1fr;
            }

            .finger-grid {
                grid-template-columns: repeat(5, minmax(138px, 1fr));
            }

            .finger-card {
                min-width: 138px;
            }

            .tactile-grid {
                grid-template-columns: repeat(5, minmax(112px, 1fr));
            }
        }

        @media (max-width: 720px) {
            .top-appbar {
                min-height: auto;
                padding-top: 12px;
                padding-bottom: 12px;
                flex-wrap: wrap;
            }

            .brand-title {
                font-size: 20px;
            }

            .header-trajectory {
                flex: 0 0 100%;
                width: 100%;
                justify-content: flex-start;
            }

            .trajectory-field {
                width: 100%;
                min-width: 0;
                flex: 1;
            }

            .hand-card-header, .runtime-inline {
                grid-template-columns: 1fr;
            }

            .setup-grid {
                grid-template-columns: 1fr;
            }

            .library-action-group {
                grid-template-columns: minmax(0, 1fr) 128px;
            }

            .preset-scroll {
                max-height: 300px;
            }

            .sequence-chip {
                grid-column: 1 / -1;
            }

            .command-console {
                left: 12px;
                right: 12px;
                bottom: 12px;
                width: auto;
            }

            .header-trajectory .q-field {
                min-width: 0 !important;
            }
        }
        </style>
        """
    )
