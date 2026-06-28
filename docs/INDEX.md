# 文档索引

本目录包含 `26-v2-r2-ind-mcu-chassis` 项目的完整文档。

## 文档导航

| 文档 | 目标读者 | 内容 |
|------|----------|------|
| [README.md](README.md) | 所有人 | 项目概述、职能总览、启动流程、关键模块介绍、构建指南 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 开发者 | 系统架构、分层设计、启动流程、实时调度、硬件映射、模块关系 |
| [AGENTS.md](AGENTS.md) | AI 编码代理 | 项目约定、代码风格、运行流程、STM32+CubeMX 规范、提交规范 |
| [upper_host_command_table.md](upper_host_command_table.md) | 上位机开发者 | 下行控制帧的完整协议规范：帧格式、命令列表、缩放规则、各命令详细说明 |
| [upper_host_feedback_table.md](upper_host_feedback_table.md) | 上位机开发者 | 上行反馈帧的完整协议规范：帧格式、ActionState 解码、Connection 位表 |
| [CHANGELOG.md](CHANGELOG.md) | 所有人 | 按月份分组的变更记录，从 2026-03 至今 |
| [PLANNING.md](PLANNING.md) | 轨迹规划开发者 | 离线轨迹规划工具的使用说明（`planning/` 子项目） |

## 文档维护规则

- 当协议变更时，需同步更新 `upper_host_command_table.md` 和 `upper_host_feedback_table.md`
- 当项目架构或结构变更时，需同步更新 `ARCHITECTURE.md`
- 当代码约定或流程变更时，需同步更新 `AGENTS.md`
- 功能增删改时，需同步更新 `CHANGELOG.md`
- `README.md` 应始终保持为项目的准确总览
