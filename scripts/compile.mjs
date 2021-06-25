import fs from 'fs';
import { fileURLToPath } from 'url';

const IMPORT_REGEX = /(from .* import .*)|(import .*)/gm;
const IMPORT_IGNORE_REGEX = /((from controller import \*)|(from|import) .*(#( |)li))/gm;
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

let OUT = ''

for (let source of SOURCES) {
    if (!EXTENSTION_REGEX.test(source)) source += '.py';

    const RAW = fs.readFileSync(fileURLToPath(new URL(source, srcDir)), { encoding: 'utf-8' });
    const RAW_PRE = RAW
        .replace(IMPORT_IGNORE_REGEX, '')
        .replace(PATHING_REGEX, '');

    for (const imprt of RAW_PRE.match(IMPORT_REGEX) ?? []) imports.add(imprt);

    const END = RAW_PRE
        .replace(IMPORT_REGEX, '');

    OUT += END;
}

fs.writeFileSync(fileURLToPath(new URL('../main.py', srcDir)), `${[...imports.values()].map((impr) => `${impr}\n`).toString().replace(/,/gm, '')}\n${OUT}`);
