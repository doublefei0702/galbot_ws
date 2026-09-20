# SPDX-License-Identifier: Apache-2.0
"""Galbot S1 MobilityGen 集成包。

启用扩展时: __init__ 导入 IExt 子类(kit 据此调用 on_startup → 注册机器人
并程序化启用 MobilityGen UI)并注册 GalbotS1Robot 到 ROBOTS registry。
"""
from .extension import *  # noqa: F401,F403  暴露 IExt 给 kit
from .robots import GalbotS1Robot  # noqa: F401
