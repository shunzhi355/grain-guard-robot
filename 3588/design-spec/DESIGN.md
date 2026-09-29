# Grain Sampling Robot — Design System

## Brand

| 项目 | 值 |
|---|---|
| 产品 | 粮食扦样机器人控制台 |
| 风格 | Dark industrial, high-contrast, professional |
| 目标设备 | 7 寸触摸屏，1920×1080 @ 96DPI |
| 系统 | ARM Linux (Orange Pi 5 Max), PySide2/Qt5 |
| 字体 | Noto Sans CJK SC → AR PL UKai CN → Noto Sans CJK HK → sans-serif |

## Colors

| Token | Hex | Usage |
|---|---|---|
| `bg-deepest` | `#0D1117` | Window background |
| `bg-dark` | `#161B22` | Cards, group boxes |
| `bg-medium` | `#21262D` | Button default, input fields |
| `border` | `#484F58` | Borders, separators |
| `accent` | `#2B579A` | Primary button, selection highlight |
| `accent-hover` | `#3668B0` | Primary button hover |
| `info` | `#58A6FF` | Info button, links |
| `success` | `#2EA043` | Start, confirm, positive |
| `warning` | `#D4A72C` | Pause, finish task |
| `danger` | `#DA3633` | Stop, emergency, errors |
| `text-primary` | `#E6EDF3` | Body text |
| `text-secondary` | `#8B949E` | Subtle text, disabled but readable |

## Typography

| 层级 | 字号 | 粗细 | 用途 |
|---|---|---|---|
| 页面标题 | 14-16pt | Bold | Page name |
| 区块标题 | 12-13pt | Bold | Section headers |
| 正文 | 8-10pt | Regular | Labels, body |
| 按钮文字 | 10-12pt | Bold | Button labels |
| 小字/辅助 | 7-8pt | Regular | Timestamps, hints |

⚠️ 所有文字必须使用板端已安装的中文字体（Noto Sans CJK SC），不可使用 Windows 专属字体（如 Microsoft YaHei）。

## Components

### Button
| 状态 | 背景 | 文字色 | 边框 |
|---|---|---|---|
| Normal | `#21262D` | `#E6EDF3` | `#484F58` |
| Hover | `#30363D` | `#E6EDF3` | `#58A6FF` |
| Pressed | `#2B579A` | `#E6EDF3` | `#2B579A` |
| Disabled | `#161B22` | `#8B949E` (must be readable) | `#30363D` |
| Primary | `#2B579A` | `#FFFFFF` | — |
| Info | `#58A6FF` | `#FFFFFF` | — |
| Success | `#2EA043` | `#FFFFFF` | — |
| Warning | `#D4A72C` | `#FFFFFF` | — |
| Danger | `#DA3633` | `#FFFFFF` | — |

- min-height: 16-22pt
- padding: 4px 10px
- border-radius: 8px

### Card
- background: `#161B22`
- border: 1px solid `#484F58`
- border-radius: 6px
- padding: 4-6px

### Input
- background: `#21262D`
- text: `#E6EDF3`
- border: 1px solid `#484F58`
- border-radius: 4px

## Constraints

- 适配 7 寸触摸屏（物理面积约 155×87mm），按钮区域 ≤ 20mm 高度方便手指触摸
- 窗口整体尺寸不变，通过缩小内部元素适配
- 禁用态文字必须与背景有足够对比度（≥ 4.5:1）
- 所有中文使用 Noto Sans CJK SC 或 AR PL UKai CN
