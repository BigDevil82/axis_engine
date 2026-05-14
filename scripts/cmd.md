uv run python cad_tests\plot_structural_design_from_dxf.py --wall-layer WALL --opening-layer WINDOW --axis-layer DOTE --show --source-backend dxf --slab-max-edge 6000 --dxf E:\Common\Desktop\test\ai-structures\case3\tod.dxf

uv run python cad_tests\plot_windows_from_dxf.py --wall-layer WALL --opening-layer WINDOW --axis-layer DOTE --show --source-backend dxf --dxf E:\Common\Desktop\test\ai-structures\case3\tod.dxf

uv run python .\cad_tests\plot_wall_axes_from_dxf.py --wall-layer WALL --opening-layer WINDOW --axis-layer DOTE --show --dxf E:\Common\Desktop\test\ai-structures\case3\tod.dxf