"""Check experiment outcomes, video decodability and report text/rendering."""
import json
from pathlib import Path
import imageio.v2 as imageio
import pypdfium2 as pdfium
from pypdf import PdfReader
from PIL import Image,ImageOps,ImageDraw
ROOT=Path(__file__).resolve().parent

def main():
    out=ROOT/'outputs';review=out/'verification';review.mkdir(exist_ok=True)
    rows=json.loads((out/'reproduction/summary.json').read_text())
    assert len(rows)==22 and all(r['matches_expectation'] for r in rows)
    integrated=[json.loads((out/'reproduction'/r['result']).read_text()) for r in rows if r['case'].startswith('integrated_')]
    assert len(integrated)==9
    assert all(r['guidance_passed'] and r['wait_count']==r['resume_count']==1 for r in integrated)
    baseline=json.loads((out/'reproduction/integrated_p8_person_aware_seed0.json').read_text())
    filmed=json.loads((out/'integrated_demo_person_aware_seed0.json').read_text())
    assert filmed['samples']==baseline['samples'],'Render changed simulation trajectory'
    reader=imageio.get_reader(out/'final_demo.mp4')
    meta=reader.get_meta_data();frames=reader.count_frames()
    declared=json.loads((out/'final_demo_metadata.json').read_text())
    assert frames==declared['frames'] and meta['size']==(1280,720)
    assert 60<=frames/meta['fps']<=180
    sheet=Image.new('RGB',(960,810),'white');draw=ImageDraw.Draw(sheet)
    for j,t in enumerate([2,16,34,49,76,98]):
        im=Image.fromarray(reader.get_data(round(t*25))).resize((480,270))
        sheet.paste(im,((j%2)*480,(j//2)*270))
    sheet.save(review/'video_contact_sheet.png');reader.close()
    pdf=ROOT/'output/pdf/physical_ai_final_report.pdf'
    pdfreader=PdfReader(pdf)
    assert len(pdfreader.pages)==4
    assert all(len(p.extract_text())>200 for p in pdfreader.pages)
    assert 'ロボット犬' in pdfreader.pages[0].extract_text()
    document=pdfium.PdfDocument(pdf)
    sheet=Image.new('RGB',(840,1188),'#cccccc')
    for i,page in enumerate(document):
        im=page.render(scale=1.5).to_pil();im.save(review/f'report_page_{i+1}.png')
        sheet.paste(ImageOps.fit(im,(420,594)),((i%2)*420,(i//2)*594))
    sheet.save(review/'report_contact_sheet.png')
    result={'experiment_trials':len(rows),'integrated_passes':len(integrated),'all_expected_outcomes':True,'render_trajectory_identical':True,'video_frames':frames,'video_duration_seconds':frames/meta['fps'],'pdf_pages':len(pdfreader.pages)}
    (review/'validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))

if __name__=='__main__':main()
