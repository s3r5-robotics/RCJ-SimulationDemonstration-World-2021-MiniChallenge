import fs from 'fs';
import { fileURLToPath } from 'url';

const IMPORT_REGEX = /(from .* import .*)|(import .*)/gm;
const IMPORT_IGNORE_REGEX = /(from|import) .*(#( |)li)/gm;
const PATHING_REGEX = /sys\.path\.append\((\n|)(.*)\)/gm;
const EXTENSTION_REGEX = /.*\.py/gm;

const SOURCES = [
    'UtilityFunctions',
    'StateMachines',
    'RobotLayer',
    'AbstractionLayer',
    'DistanceSensor',
    'Analysis',
    'FinalCode'
];

const imports = new Set();
const srcDir = new URL('../src/', import.meta.url);

let OUT = '';
let OUT_HEADER = '';

OUT_HEADER += '# https://github.com/s3r5-robotics/SimulationDemonstration-2021-MiniChallenge';
OUT_HEADER += '\n';

const generateImportHeader = () => [...imports.values()].map((impr) => `${impr}\n`).toString().replace(/,/gm, '');

for (let source of SOURCES) {
    if (!EXTENSTION_REGEX.test(source)) source += '.py';

    const RAW = fs.readFileSync(fileURLToPath(new URL(source, srcDir)), { encoding: 'utf-8' });
    const RAW_PRE = RAW
        .replace(IMPORT_IGNORE_REGEX, '')
        .replace(PATHING_REGEX, '');

    for (const imprt of RAW_PRE.match(IMPORT_REGEX) ?? []) imports.add(imprt);

    const END = RAW_PRE
        .replace(IMPORT_REGEX, '');

    OUT += `# File: "${source}"\n`;
    OUT += END;
}

fs.writeFileSync(fileURLToPath(new URL('../main.py', srcDir)), [
    OUT_HEADER,
    generateImportHeader(),
    OUT
].join('\n'));
