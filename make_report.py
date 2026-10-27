"""Create figures and a four-page Japanese report from recorded experiments.

Requires requirements-report.txt. The font defaults to a macOS-installed font;
use --font with another Japanese TrueType font on other systems.
"""
import argparse
import json
import os
from pathlib import Path
from xml.sax.saxutils import escape
ROOT=Path(__file__).resolve().parent
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'outputs/.matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, PageBreak, Table, TableStyle
from reportlab.lib.pagesizes import A4

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--font',default='/System/Library/Fonts/Supplemental/Arial Unicode.ttf');a=parser.parse_args()
    out=ROOT/'outputs';results=out/'reproduction'
    aware=json.loads((results/'integrated_p8_person_aware_seed0.json').read_text())
    unaware=json.loads((results/'no_wait_person_aware_seed0.json').read_text())
    fig,ax=plt.subplots(figsize=(8.8,3.6),layout='constrained')
    for d,label,color in [(unaware,'Waiting disabled','#d36a39'),(aware,'Person-aware waiting','#147d85')]:
        ax.plot([s['t'] for s in d['samples']],[s['gap_m'] for s in d['samples']],label=label,color=color,lw=2)
    ax.axvspan(8,14,color='gray',alpha=.15,label='Scripted person pause')
    ax.axhline(1.45,color='#555555',ls='--',lw=1,label='Evaluation threshold (1.45 m)')
    ax.set(xlabel='Simulation time (s)',ylabel='Robot-person centre distance (m)',xlim=(0,53),ylim=(.7,2))
    ax.legend(fontsize=9,loc='upper right');ax.grid(alpha=.18)
    fig.savefig(out/'gap_comparison.png',dpi=180);fig.savefig(out/'gap_comparison.svg');plt.close(fig)
    pdfmetrics.registerFont(TTFont('JP',a.font))
    ink=colors.HexColor('#192d3b');accent=colors.HexColor('#107c82')
    body=ParagraphStyle('Body',fontName='JP',fontSize=9.5,leading=15,spaceAfter=8,textColor=ink,wordWrap='CJK')
    small=ParagraphStyle('Small',parent=body,fontSize=8,leading=12)
    h1=ParagraphStyle('H1',parent=body,fontSize=20,leading=29,spaceAfter=16,textColor=accent)
    h2=ParagraphStyle('H2',parent=body,fontSize=12,leading=18,spaceBefore=9,spaceAfter=7,textColor=accent)
    story=[]
    def p(s,style=body):story.append(Paragraph(escape(s),style))
    def h(s):p(s,h2)
    def table(rows,widths):
        t=Table([[Paragraph(escape(str(c)),small) for c in row] for row in rows],colWidths=widths,hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),colors.HexColor('#dceeee')),('VALIGN',(0,0),(-1,-1),'TOP'),('BOTTOMPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),7),('LINEBELOW',(0,0),(-1,0),.6,accent),('LINEBELOW',(0,1),(-1,-1),.25,colors.HexColor('#d5dfe5'))]))
        story.append(t);story.append(Spacer(1,8))
    p('ロボット犬は\n人を連れて歩けるか？'.replace('\n',' '),h1)
    p('Physical AI 応用編1 最終課題 / 2026年10月27日',small)
    h('問い：歩けることと、案内できることの違い')
    p('低い梁をロボットだけなら通れても、人は接触する。人が立ち止まったときには、置き去りにせず待つ必要もある。この2つの問題を、四足歩行ロボットGo2と簡易人モデルで検証した。')
    story.append(Image(str(out/'clearance_comparison.png'),width=495,height=185.625))
    p('同じ出入口の比較。左は人が梁へ接触して試行終了。右は人の高さも考慮して迂回。',small)
    h('構成と実行環境')
    p('既知地図 → A*経路計画 → 位置・向きのフィードバック → 学習済み歩行モデル → 関節目標角・PD制御 → MuJoCo物理演算。人の観測速度と距離から、待機・再開を切り替える。')
    p('AIは公開済みのGo2歩行モデルをCPUで推論する。経路計画と待機判断は明示的なアルゴリズムであり、新しいモデルの学習は行っていない。ロボットを直接経路上へ移動させる位置書き換えは使わない。')
    p('Apple M3 / 16GB / macOS 26.5.2 / Python 3.12 / MuJoCo 3.2.7 / PyTorch 2.5.1。物理刻み0.002秒、推論と案内判断0.02秒。独立環境を作り直して再検証。実機・クラウドGPUは使用しない。')
    h('この実験で得られた結果')
    p('迂回と待機・再開を統合した9試験で、接触・転倒なく両者が到着した。ただし、既知地図・理想観測・簡易人モデルでの基礎検証であり、実利用の安全性や盲導犬の代替を示すものではない。')
    story.append(PageBreak())
    p('比較実験と結果',h1)
    p('統合試験は停止開始8・20・32秒 × 微小な初期状態変更3条件。停止時間は各6秒。同じ地図での9試験であり、実環境の成功率を推定する統計試験ではない。')
    table([['方式・検証','試行数','結果'],['直進・旋回・保持 / 直線同行 / 迂回','各3','各段階の基準を満たす'],['ロボットのみの高さで計画','1','13.162秒：人が梁に接触'],['迂回余裕を0.32mに設定','1','25.102秒：人が柱に接触'],['迂回＋待機・再開','9','全条件で案内成功'],['待機を無効化','1','到着するが最大距離1.865m'],['人を停止させない','1','到着、不要な待機0回']],[230,48,217])
    p('全22試行：成功を想定した19試行が成功。比較3試行では想定した接触または距離超過。固定アセット29ファイルと人形状距離33ケースも照合。',small)
    story.append(Image(str(out/'gap_comparison.png'),width=495,height=202.5))
    p('基準条件の中心間距離。灰色は人への停止指示8〜14秒。待機なしは最大1.865m、統合方式は1.287m。1.45mは本実験の評価閾値であり安全規格ではない。',small)
    p('統合9試験：最小すき間19.22〜20.65cm、最大距離1.142〜1.292m、待機中変位2.59〜3.26cm。各1回の待機・再開。終了51.46〜52.40秒は到着後5秒の保持を含む。')
    story.append(PageBreak())
    p('判定方法と失敗からの学び',h1)
    h('人モデルと衝突の扱い')
    p('幅0.44m・高さ1.8mの人を胴体カプセルと頭球で表現。ロボットの実軌跡を記録し、約0.9m後方を最大0.22m/s、加速度上限0.5m/s²で追従する。人体の歩行・バランス・リードの張力は計算しない。')
    p('人と障害物は形状の交差を検査し、接触時点で試行を終了する。ロボットはMuJoCoの物理接触を検査する。目的地は人の外形全体も入れる広い領域であり、人の一点目標への到着ではない。')
    h('成功の定義')
    p('ロボットが終点から8cm以内、人の外形全体がx=2.5〜4.5m・y=±0.85mの目的地内、接触・転倒なし、中心間距離1.45m未満。停止条件では待機と再開が成立し、待機開始後の停止期間内変位10cm未満。')
    h('計画経路と実行軌跡は違う')
    p('水平余裕0.32mでは、計画上は迂回できても歩行軌跡の内側へのずれで人が柱に接触した。余裕0.50mでこの環境の接触は解消した。失敗ログも残し、都合のよい動画だけで評価しない構成にした。')
    h('到着だけでは同行の質が分からない')
    p('待機なしでも最後には到着するが、途中の距離は1.865mに開く。案内の評価には、人の身体の通過可能性・距離・停止への応答を含める必要がある。')
    h('待機の遅れは残る')
    p('判断器は停止予定を読まず、観測速度と距離を使う。停止指示から待機まで2.08〜2.96秒かかった。減速・観測平滑化・距離条件を含む値で、完全静止後の検出時間ではない。不要な待機を減らす条件と応答の速さにはトレードオフがある。')
    h('未検証の範囲')
    p('未知地図、カメラ・LiDAR、位置推定誤差、移動障害物、段差、人の横ずれや触覚情報は未実装。人は軌跡を正確に追う強い仮定がある。初期状態の微小な変更だけで、未知環境への汎化は確認していない。')
    story.append(PageBreak())
    p('再現・学習・次の実験',h1)
    h('保存した成果物')
    table([['ファイル','用途'],['REPORT.md','詳細レポートと限界・参考文献'],['IMPLEMENTATION.md','環境準備、段階ごとの実行方法、指標の読み方'],['LEARNING_GUIDE.md','本人が60分で現象とコードを確かめる実習'],['run_experiments.py','全比較の一括再実行。失敗比較も検証'],['outputs/reproduction/','試行別JSON、集計CSV、時系列とイベント'],['outputs/final_demo.mp4','約102秒の字幕入り説明動画。音声なし'],['manifest / THIRD_PARTY.md','固定コミット、ハッシュ、出所とライセンス']],[190,305])
    h('エージェント利用と本人による確認')
    p('利用者がテーマと制約を決め、実装・実行・ログ整理・文章化はCodexを利用した。ここでの考察はログに基づく分析であり、本人が理解・体験したという記述ではない。学習ガイドで追試し、本人の予想との差や次に研究したい点を追記できる。')
    h('次に調べたいこと')
    p('人の速度・横ずれ・停止時間と未知の障害物配置を変え、閾値を固定して評価する。その後に知覚誤差とハーネス力学を追加する。案内を強化学習へ発展させる際も、到着報酬だけでなく人の接触・離れ過ぎ・急な加減速を扱う必要がある。')
    h('提出状態')
    p('元の課題PDFのOneDriveパスは今回アクセスできず、原本との最終照合は未完了。YouTubeへの投稿、本人の理解確認、正式な提出は別工程。完全な実験結果はREPORT.mdとJSONを参照。')
    h('参考資料・出所')
    for s in ['[1] Kim et al., CoRL 2023: https://arxiv.org/abs/2306.14055','[2] Policy: https://github.com/Teddy-Liao/walk-these-ways-go2','[3] Model: https://github.com/google-deepmind/mujoco_menagerie','[4] Controller reference: https://github.com/art-e-fact/unitree_mujoco']:
        p(s,small)
    p('先行研究[1]の実測人モデルや実機ハーネスを再現したものではない。学習済み重みと形状の著作権・ライセンスは各配布元に帰属し、同梱した通知を保持する。',small)
    dest=ROOT/'output/pdf';dest.mkdir(parents=True,exist_ok=True)
    def footer(canvas,doc):
        canvas.setStrokeColor(accent);canvas.line(50,42,545,42)
        canvas.setFont('JP',8);canvas.setFillColor(ink)
        canvas.drawString(50,28,'PHYSICAL AI / GUIDE DOG SIMULATION / 2026-10-27')
        canvas.drawRightString(545,28,str(doc.page))
    doc=SimpleDocTemplate(str(dest/'physical_ai_final_report.pdf'),pagesize=A4,rightMargin=50,leftMargin=50,topMargin=42,bottomMargin=54,title='ロボット犬は人を連れて歩けるか？',author='Physical AI project')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    from pypdf import PdfReader
    reader=PdfReader(dest/'physical_ai_final_report.pdf')
    assert len(reader.pages)==4, f'Expected four pages, got {len(reader.pages)}'
    assert all(len(p.extract_text())>200 for p in reader.pages)
    print('Report verified:',len(reader.pages),'pages')

if __name__=='__main__':main()
