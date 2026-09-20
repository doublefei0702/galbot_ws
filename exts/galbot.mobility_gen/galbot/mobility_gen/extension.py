# SPDX-License-Identifier: Apache-2.0
"""扩展启动钩子。

关键顺序: 必须先让 galbot.mobility_gen 模块完成 ROBOTS 注册,
再启用 MobilityGen UI —— UI 在自己的 on_startup 里构建机器人下拉框,
晚于我们的注册, GalbotS1Robot 才会出现在列表里。
"""
import omni.ext


class GalbotMobilityGenExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        import galbot.mobility_gen  # noqa: F401  触发 @ROBOTS.register()

        from omni.kit.app import get_app

        em = get_app().get_extension_manager()
        em.set_extension_enabled_immediate("isaacsim.replicator.mobility_gen.ui", True)
        print("[galbot.mobility_gen] 已注册 GalbotS1Robot 并启用 MobilityGen UI")
