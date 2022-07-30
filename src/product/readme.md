# 生成物たちのフォルダ
**加速度は未実装なので0が入ってる**

productフォルダの中身はros2とは関係ない<br>
|ファイル名|内容|
|---|---|
|movie/pyroute.py|動画生成用の経路データ|
|movie/route.mp4|動画データ|
|move_targets.cpp|経路のMoveTargetたち|
|array.cpp|経路のarrayたち|

# 動画生成
pyroute.pyに経路吐いて
```bash
python3 src/calc_route/src/product/movie/movie_generator.py 
```
で動画の生成ができる