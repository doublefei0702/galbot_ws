# SPDX-License-Identifier: Apache-2.0
"""扩展启动钩子。

关键顺序: 必须先让 galbot.mobility_gen 模块完成 ROBOTS 注册,
再启用 MobilityGen UI —— UI 在自己的 on_startup 里构建机器人下拉框,
晚于我们的注册, GalbotS1Robot 才会出现在列表里。
"""
import omni.ext


def _save_stage_with_absolute_asset_paths(usd_path: str, save_and_reload_in_place: bool = True) -> bool:
    """Save a MobilityGen scene without location-dependent asset references.

    Isaac Sim's stock ``save_stage`` stores rewritten asset paths relative to
    its temporary cache layer.  MobilityGen later copies that layer into each
    recording, so those paths point somewhere else during replay (especially
    when the recording is selected through a symlink).  Store absolute paths
    instead so the recording can be opened from any directory.
    """
    import omni.usd
    from isaacsim.core.utils.stage import get_current_stage, open_stage
    from pxr import Sdf, Usd

    if not Usd.Stage.IsSupportedFile(usd_path):
        raise ValueError("Only USD files can be saved with this method")

    layer = Sdf.Layer.CreateNew(usd_path)
    root_layer = get_current_stage().GetRootLayer()
    layer.TransferContent(root_layer)
    # store_relative_path=False is the essential difference from the stock
    # isaacsim.core.utils.stage.save_stage implementation.
    omni.usd.resolve_paths(root_layer.identifier, layer.identifier, False)
    result = layer.Save()
    if save_and_reload_in_place:
        open_stage(usd_path)
    return result


class GalbotMobilityGenExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        import galbot.mobility_gen  # noqa: F401  触发 @ROBOTS.register()

        from omni.kit.app import get_app

        em = get_app().get_extension_manager()
        em.set_extension_enabled_immediate("isaacsim.replicator.mobility_gen.ui", True)

        # mobility_gen.ui imports save_stage into its own module namespace, so
        # patch that binding after enabling the UI.  Future recordings then
        # remain valid after the cached stage is copied or opened via symlink.
        from isaacsim.replicator.mobility_gen.ui import extension as mobility_gen_ui_extension

        mobility_gen_ui_extension.save_stage = _save_stage_with_absolute_asset_paths
        print("[galbot.mobility_gen] 已注册 GalbotS1Robot，启用 MobilityGen UI，并启用绝对资产路径保存")
