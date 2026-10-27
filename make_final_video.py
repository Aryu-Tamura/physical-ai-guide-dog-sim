"""Build a captioned ~100-second silent explainer from actual simulation frames."""
from pathlib import Path
import argparse
import json
import numpy as np
import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont
ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--font',default='/System/Library/Fonts/ヒラギノ角ゴシック W3.ttc');a=parser.parse_args()
    fontpath=Path(a.font)
    if not fontpath.exists():
        import unicodedata
        fontpath=next(p for p in Path('/System/Library/Fonts').glob('*.ttc') if unicodedata.normalize('NFC',p.name)=='ヒラギノ角ゴシック W3.ttc')
    fonts={size:ImageFont.truetype(str(fontpath),size) for size in [20,24,28,32,44,54]}
    out=ROOT/'outputs';writer=imageio.get_writer(str(out/'final_demo.mp4'),fps=25,quality=8)
    count=0
    def frame(title):
        im=Image.new('RGB',(1280,720),(17,26,37));d=ImageDraw.Draw(im)
        d.text((48,30),title,font=fonts[32],fill='white')
        d.rectangle((48,88,1232,92),fill=(56,190,179))
        return im,d
    def emit(im):
        nonlocal count
        writer.append_data(np.asarray(im));count+=1
    def card(title,lines,seconds):
        im,d=frame('PHYSICAL AI / シミュレーション実験')
        d.text((60,170),title,font=fonts[44],fill=(87,220,202))
        for i,line in enumerate(lines):d.text((60,290+i*65),line,font=fonts[28],fill='white')
        for _ in range(round(seconds*25)):emit(im)
    try:
        card('ロボット犬は、人を連れて歩けるか？',['ロボットが通れる道でも、人が通れるとは限らない。','MacのCPUで、学習済み歩行AI＋人を考慮した案内を検証。','実機なし / 人は単純なモデル / 字幕のみ'],8)
        reader=imageio.get_reader(out/'clearance_comparison.mp4')
        for i,raw in enumerate(reader):
            if i%2:continue
            im,d=frame('01 / 同じ出入口：人の高さを考慮すると、経路が変わる')
            im.paste(Image.fromarray(raw),(0,120))
            d.text((48,630),'左：人が梁に接触し終了   右：低い梁を迂回して到着',font=fonts[28],fill='white')
            d.text((48,675),'映像は2倍速。左画面は接触時点で停止。人の接触は形状の交差で判定。',font=fonts[20],fill=(180,196,210))
            emit(im)
        reader.close()
        card('次の問い：途中で人が止まったら？',['人の停止予定は、案内の判断器には渡さない。','観測した速度と距離から、待機・再開を決める。','次は実時間の統合実験。'],6)
        reader=imageio.get_reader(out/'integrated_demo_person_aware_seed0.mp4')
        for i,raw in enumerate(reader):
            t=i/25
            im,d=frame('02 / 人の停止 → 待機 → 再開 → 迂回して一緒に到着')
            im.paste(Image.fromarray(raw),(48,130))
            if t<8:label='人と一緒に進む';body=['黄色：計画した経路','水色：人モデル','緑色：目的地領域']
            elif t<10.12:label='人が停止';body=['人は8〜14秒に停止','ロボットは速度と','距離の変化を観測']
            elif t<14.20:label='その場で待機';body=['歩行AIで位置を保持','足踏みは続く','人の再開を待つ']
            elif t<47.3:label='案内を再開';body=['人の移動を観測して復帰','低い梁の外側へ迂回','両者の接触を確認']
            else:label='両者が目的地へ';body=['ロボットだけでなく','人も緑の領域へ到着','接触・転倒なし']
            d.text((740,170),label,font=fonts[32],fill=(87,220,202))
            for j,line in enumerate(body):d.text((740,250+j*55),line,font=fonts[24],fill='white')
            d.text((48,650),'実時間 / 既知の地図と理想的な位置観測 / 人は記録軌跡を追従する簡易モデル',font=fonts[20],fill=(180,196,210))
            emit(im)
        reader.close()
        card('歩けることから、人を案内することへ',['学習済みAIの担当：四足歩行。経路計画と待機は明示的な制御。','実機・実際の利用者・センサ誤差・リードの力は未検証。','コード、失敗例、数値ログ、再現手順をセットで公開。'],12)
    finally:writer.close()
    (out/'final_demo_metadata.json').write_text(json.dumps({'fps':25,'frames':count,'duration_seconds':count/25,'audio':False,'comparison_playback_speed':2,'integrated_playback_speed':1},indent=2))
    print('Saved',count/25,'seconds')

if __name__=='__main__':main()
