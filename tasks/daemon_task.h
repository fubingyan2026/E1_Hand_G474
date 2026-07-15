/*
 * Copyright (c) 2026 E1_PRO 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    daemon_task.h
 * @brief   守护进程监控任务 — 9 电机反馈超时看门狗（掉线/恢复日志告警）
 */

#ifndef DAEMON_TASK_H
#define DAEMON_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void daemon_task_init(void);

#ifdef __cplusplus
}
#endif

#endif /* DAEMON_TASK_H */
