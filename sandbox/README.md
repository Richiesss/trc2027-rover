# sandbox/ — センサの動作確認

Jetson上でセンサを1つずつ試すための場所です。本番のコード（`rover/`）とは分けて管理します。

- 何を試してもかまいませんが、うまくいった手順はNotionの技術メモに残してください。
- Pythonの仮想環境はリポジトリの外に作ります。

```bash
python3 -m venv ~/venvs/sandbox
source ~/venvs/sandbox/bin/activate
pip install -r ~/trc2027-rover/sandbox/requirements.txt
```

| フォルダ | 内容 |
| --- | --- |
| `gnss/` | GNSSの位置ログ取得、ファームウェア確認、CLAS設定のスクリプト |
| `lidar/` | Unitree L2 の公式SDK（各自で clone、コミットしない）と確認用コード |
