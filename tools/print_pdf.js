// Print an HTML file to a US Letter PDF with the preinstalled Chromium.
// Usage: node tools/print_pdf.js <in.html> <out.pdf> <footer-html>
// Called by tools/make_checklist_pdf.py.
const { chromium } = require('playwright');

(async () => {
  const [inHtml, outPdf, footer] = process.argv.slice(2);
  const browser = await chromium.launch();
  const page = await browser.newPage();
  await page.goto('file://' + inHtml, { waitUntil: 'load' });
  await page.evaluate(() => document.fonts.ready);
  await page.pdf({
    path: outPdf,
    format: 'Letter',
    printBackground: true,
    preferCSSPageSize: true,
    displayHeaderFooter: true,
    headerTemplate: '<div></div>',
    footerTemplate: footer,
    margin: { top: '0.55in', right: '0.55in', bottom: '0.6in', left: '0.55in' },
  });
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
