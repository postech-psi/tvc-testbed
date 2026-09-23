import fs from "node:fs/promises";
import { SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const root = "C:/Users/tae06/CODE/tvc-testbed/research_outputs/curated";
const data = JSON.parse(await fs.readFile(`${root}/curated_corpus.json`, "utf8"));
const wb = Workbook.create();
const overview = wb.worksheets.add("Overview");
const main = wb.worksheets.add("Main Corpus");
const inspiration = wb.worksheets.add("Method Inspiration");
const excluded = wb.worksheets.add("Excluded");
const font = "Arial";

for (const s of [overview, main, inspiration, excluded]) {
  s.showGridLines = false;
  s.getRange("A1:Z200").format.font = { name: font, size: 10, color: "#202020" };
}

// Overview
overview.getRange("A2:H2").merge();
overview.getRange("A2").values = [["Curated TVC Ashby Corpus"]];
overview.getRange("A2:H2").format.font = { name: font, size: 16, bold: true, color: "#202020" };
overview.getRange("A3:H3").merge();
overview.getRange("A3").values = [["Peer-reviewed primary studies on physically comparable thrust-vector architectures"]];
overview.getRange("A3:H3").format.font = { name: font, size: 10, italic: true, color: "#555555" };
overview.getRange("A5:B8").values = [
  ["Corpus", "Count"],
  ["Main plotted studies", data.scope.main_count],
  ["Method inspiration", data.scope.inspiration_count],
  ["Original records excluded", data.scope.excluded_count],
];
overview.getRange("A5:B5").format = { fill: "#234E70", font: { name: font, bold: true, color: "#FFFFFF" }, horizontalAlignment: "center" };
overview.getRange("A6:A8").format.font = { name: font, bold: true };
overview.getRange("D5:H5").merge();
overview.getRange("D5").values = [["Main-plot inclusion rules"]];
overview.getRange("D5:H5").format = { fill: "#234E70", font: { name: font, bold: true, color: "#FFFFFF" } };
data.scope.rules.forEach((rule, i) => { overview.getRange(`D${6+i}:H${6+i}`).merge(); overview.getRange(`D${6+i}`).values = [[`${i+1}. ${rule}`]]; });
overview.getRange("D6:H11").format.wrapText = true;
overview.getRange("D6:H11").format.rowHeight = 30;
overview.getRange("A13:H13").merge();
overview.getRange("A13").values = [["Main conclusion"]];
overview.getRange("A13:H13").format = { fill: "#D9EAF2", font: { name: font, bold: true, color: "#173B55" } };
overview.getRange("A14:H15").merge();
overview.getRange("A14").values = [["The close-architecture literature is dominated by classical, optimization, nonlinear and identification methods. Learning-based evidence exists mainly on alternate or distributed vectoring platforms, not on the near-exact single/coaxial two-axis architecture with free-flight validation."]];
overview.getRange("A14:H15").format.wrapText = true;
overview.getRange("A14:H15").format.verticalAlignment = "center";

const png = await fs.readFile(`${root}/tvc_ashby_scoped.png`);
overview.images.add({
  dataUrl: `data:image/png;base64,${png.toString("base64")}`,
  anchor: { from: { row: 16, col: 0 }, extent: { widthPx: 1100, heightPx: 688 } },
});
overview.getRange("A1:H55").format.columnWidth = 18;
overview.getRange("A:A").format.columnWidth = 28;
overview.getRange("D:H").format.columnWidth = 22;
overview.freezePanes.freezeRows(3);

// Main corpus
const mainHeaders = ["ID","Year","Paper Title","Citation","Venue","Publication Type","Architecture Class","Architecture Distance Code","Architecture","Vectoring","Control Family","Control Method","Learning Role","Functional Scope","Functional Scope Code","Validation","Validation Code","Hardware Evidence","Free Flight","Evidence Status","Key Metrics","Project Relevance","DOI","Source URL"];
const mainRows = data.main.map(r => [r.ID,r.Year,r.Title,r.Citation,r.Venue,r["Publication Type"],r["Architecture Class"],r["Architecture Distance Code"],r.Architecture,r.Vectoring,r["Control Family"],r["Control Method"],r["Learning Role"],r["Functional Scope"],r["Functional Scope Code"],r.Validation,r["Validation Code"],r["Hardware Evidence"],r["Free Flight"],r["Evidence Status"],r["Key Metrics"],r["Project Relevance"],r.DOI,r["Source URL"]]);
main.getRange("A2:X2").values = [mainHeaders];
main.getRange("A3").write(mainRows);
main.getRange(`A2:X${2+mainRows.length}`).format.verticalAlignment = "top";
main.getRange(`A2:X${2+mainRows.length}`).format.wrapText = true;
main.getRange(`A3:X${2+mainRows.length}`).format.rowHeight = 62;
main.getRange("A2:X2").format = { fill: "#234E70", font: { name: font, bold: true, color: "#FFFFFF" }, horizontalAlignment: "center", verticalAlignment: "center" };
main.tables.add(`A2:X${2+mainRows.length}`, true, "MainCorpusTable").style = "TableStyleMedium2";
main.getRange("A1:X1").merge(); main.getRange("A1").values = [["Main Ashby corpus — only these studies are plotted"]];
main.getRange("A1:X1").format.font = { name: font, size: 14, bold: true };
main.freezePanes.freezeRows(2); main.freezePanes.freezeColumns(2);
main.getRange("A:A").format.columnWidth = 9; main.getRange("B:B").format.columnWidth = 9;
main.getRange("C:C").format.columnWidth = 42; main.getRange("D:F").format.columnWidth = 20;
main.getRange("G:J").format.columnWidth = 26; main.getRange("K:N").format.columnWidth = 27;
main.getRange("O:Q").format.columnWidth = 14; main.getRange("R:T").format.columnWidth = 18;
main.getRange("U:V").format.columnWidth = 35; main.getRange("W:X").format.columnWidth = 28;

// Method inspiration
const inspHeaders = ["ID","Year","Paper Title","Control Method","Validation","Why outside main plot","Source URL"];
const inspRows = data.inspiration.map(r => [r.ID,r.Year,r.Title,r["Control Method"],r.Validation,r["Why outside main plot"],r["Source URL"]]);
inspiration.getRange("A2:G2").values = [inspHeaders]; inspiration.getRange("A3").write(inspRows);
inspiration.getRange("A1:G1").merge(); inspiration.getRange("A1").values = [["Method inspiration — useful ideas, not direct Ashby evidence"]];
inspiration.getRange("A1:G1").format.font = { name: font, size: 14, bold: true };
inspiration.getRange("A2:G2").format = { fill: "#6B5B73", font: { name: font, bold: true, color: "#FFFFFF" }, horizontalAlignment: "center" };
inspiration.tables.add(`A2:G${2+inspRows.length}`, true, "MethodInspirationTable").style = "TableStyleMedium4";
inspiration.getRange("A:G").format.wrapText = true; inspiration.getRange("A:G").format.verticalAlignment = "top";
inspiration.getRange(`A3:G${2+inspRows.length}`).format.rowHeight = 68;
inspiration.getRange("A:B").format.columnWidth = 10; inspiration.getRange("C:C").format.columnWidth = 48;
inspiration.getRange("D:F").format.columnWidth = 36; inspiration.getRange("G:G").format.columnWidth = 35;
inspiration.freezePanes.freezeRows(2);

// Exclusions
const exHeaders = ["ID","Year","Paper Title","Publication Type","Original Relevance Ring","Exclusion Reason","Source URL"];
const exRows = data.excluded.map(r => [r.ID,r.Year,r.Title,r["Publication Type"],r["Relevance Ring"],r.Reason,r["Source URL"]]);
excluded.getRange("A2:G2").values = [exHeaders]; excluded.getRange("A3").write(exRows);
excluded.getRange("A1:G1").merge(); excluded.getRange("A1").values = [["Excluded from the main Ashby plot — retained for auditability"]];
excluded.getRange("A1:G1").format.font = { name: font, size: 14, bold: true };
excluded.getRange("A2:G2").format = { fill: "#5A5A5A", font: { name: font, bold: true, color: "#FFFFFF" }, horizontalAlignment: "center" };
excluded.tables.add(`A2:G${2+exRows.length}`, true, "ExcludedTable").style = "TableStyleMedium3";
excluded.getRange("A:G").format.wrapText = true; excluded.getRange("A:G").format.verticalAlignment = "top";
excluded.getRange(`A3:G${2+exRows.length}`).format.rowHeight = 38;
excluded.getRange("A:B").format.columnWidth = 10; excluded.getRange("C:C").format.columnWidth = 48;
excluded.getRange("D:F").format.columnWidth = 28; excluded.getRange("G:G").format.columnWidth = 38;
excluded.freezePanes.freezeRows(2);

wb.recalculate();
const inspect = await wb.inspect({ kind: "table", sheetId: "Main Corpus", range: `A1:X${2+mainRows.length}`, include: "values,formulas", tableMaxRows: 16, tableMaxCols: 24, maxChars: 12000 });
console.log(inspect.ndjson);
const errors = await wb.inspect({ kind: "match", searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!", options: { useRegex: true, maxResults: 100 }, summary: "final formula error scan" });
console.log(errors.ndjson);
const preview = await wb.render({ sheetName: "Overview", range: "A1:H55", scale: 1, format: "png" });
await fs.writeFile(`${root}/workbook_overview_preview.png`, new Uint8Array(await preview.arrayBuffer()));
for (const [sheetName, range, fileName] of [
  ["Main Corpus", `A1:X${2+mainRows.length}`, "preview_main_corpus.png"],
  ["Method Inspiration", `A1:G${2+inspRows.length}`, "preview_method_inspiration.png"],
  ["Excluded", `A1:G${2+exRows.length}`, "preview_excluded.png"],
]) {
  const rendered = await wb.render({ sheetName, range, scale: 1, format: "png" });
  await fs.writeFile(`${root}/${fileName}`, new Uint8Array(await rendered.arrayBuffer()));
}
const output = await SpreadsheetFile.exportXlsx(wb);
await output.save(`${root}/TVC_Ashby_Curated_Corpus.xlsx`);
console.log(`Saved ${root}/TVC_Ashby_Curated_Corpus.xlsx`);
