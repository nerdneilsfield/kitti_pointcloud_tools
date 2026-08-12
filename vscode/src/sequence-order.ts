type TokenKind = "text" | "number";

interface SequenceToken {
  kind: TokenKind;
  value: string;
}

class SequenceNameScanner {
  private cursor = 0;

  constructor(private readonly name: string) {}

  get empty(): boolean {
    return this.cursor >= this.name.length;
  }

  next(): SequenceToken {
    const start = this.cursor;
    const number = isDigit(this.name[this.cursor]);
    if (number) {
      while (isDigit(this.name[this.cursor])) ++this.cursor;
      if (this.name[this.cursor] === "." && isDigit(this.name[this.cursor + 1])) {
        ++this.cursor;
        while (isDigit(this.name[this.cursor])) ++this.cursor;
      }
    } else {
      while (!this.empty && !isDigit(this.name[this.cursor])) ++this.cursor;
    }
    return {
      kind: number ? "number" : "text",
      value: this.name.slice(start, this.cursor),
    };
  }
}

function isDigit(value: string | undefined): boolean {
  return value !== undefined && value >= "0" && value <= "9";
}

function compareNumber(left: string, right: string): number {
  const [leftInteger, leftFraction = ""] = left.split(".", 2);
  const [rightInteger, rightFraction = ""] = right.split(".", 2);
  const normalizedLeftInteger = leftInteger.replace(/^0+/u, "");
  const normalizedRightInteger = rightInteger.replace(/^0+/u, "");
  if (normalizedLeftInteger.length !== normalizedRightInteger.length) {
    return normalizedLeftInteger.length - normalizedRightInteger.length;
  }
  const integerOrder = compareText(
    normalizedLeftInteger,
    normalizedRightInteger,
  );
  if (integerOrder !== 0) return integerOrder;

  const normalizedLeftFraction = leftFraction.replace(/0+$/u, "");
  const normalizedRightFraction = rightFraction.replace(/0+$/u, "");
  const length = Math.max(
    normalizedLeftFraction.length,
    normalizedRightFraction.length,
  );
  for (let index = 0; index < length; ++index) {
    const leftDigit = normalizedLeftFraction[index] ?? "0";
    const rightDigit = normalizedRightFraction[index] ?? "0";
    if (leftDigit !== rightDigit) return leftDigit < rightDigit ? -1 : 1;
  }
  return 0;
}

function compareText(left: string, right: string): number {
  return left < right ? -1 : left > right ? 1 : 0;
}

interface NumericFields {
  sequenceNumbers: string[];
  timestamps: string[];
}

function numericFields(name: string): NumericFields {
  const result: NumericFields = { sequenceNumbers: [], timestamps: [] };
  const scanner = new SequenceNameScanner(name);
  while (!scanner.empty) {
    const token = scanner.next();
    if (token.kind !== "number") continue;
    (token.value.includes(".")
      ? result.timestamps
      : result.sequenceNumbers).push(token.value);
  }
  return result;
}

function compareNumericFields(left: string[], right: string[]): number {
  const shared = Math.min(left.length, right.length);
  for (let index = 0; index < shared; ++index) {
    const order = compareNumber(left[index], right[index]);
    if (order !== 0) return order;
  }
  return left.length - right.length;
}

export function compareSequenceNames(left: string, right: string): number {
  const leftFields = numericFields(left);
  const rightFields = numericFields(right);
  // Explicit integer sequence fields outrank decimal timestamps, regardless
  // of where either field occurs in the filename.
  if (leftFields.sequenceNumbers.length && rightFields.sequenceNumbers.length) {
    const order = compareNumericFields(
      leftFields.sequenceNumbers,
      rightFields.sequenceNumbers,
    );
    if (order !== 0) return order;
  }
  if (leftFields.timestamps.length && rightFields.timestamps.length) {
    const order = compareNumericFields(
      leftFields.timestamps,
      rightFields.timestamps,
    );
    if (order !== 0) return order;
  }

  const leftScanner = new SequenceNameScanner(left);
  const rightScanner = new SequenceNameScanner(right);
  while (!leftScanner.empty && !rightScanner.empty) {
    const leftToken = leftScanner.next();
    const rightToken = rightScanner.next();
    const order = leftToken.kind === rightToken.kind
      ? leftToken.kind === "number"
        ? compareNumber(leftToken.value, rightToken.value)
        : compareText(leftToken.value, rightToken.value)
      : compareText(leftToken.value, rightToken.value);
    if (order !== 0) return order;
  }
  if (leftScanner.empty !== rightScanner.empty) {
    return leftScanner.empty ? -1 : 1;
  }
  return compareText(left, right);
}

interface CatalogNumericField {
  value: string;
  timestamp: boolean;
}

interface CatalogFieldScore {
  index: number;
  distinct: number;
  allUnique: boolean;
  integer: boolean;
  density: number;
}

function catalogNumericFields(name: string): CatalogNumericField[] {
  const fields: CatalogNumericField[] = [];
  const scanner = new SequenceNameScanner(name);
  while (!scanner.empty) {
    const token = scanner.next();
    if (token.kind === "number") {
      fields.push({ value: token.value, timestamp: token.value.includes(".") });
    }
  }
  return fields;
}

function betterFieldScore(
  left: CatalogFieldScore,
  right: CatalogFieldScore,
): boolean {
  if (left.allUnique !== right.allUnique) return left.allUnique;
  if (left.integer !== right.integer) return left.integer;
  if (left.density !== right.density) return left.density > right.density;
  if (left.distinct !== right.distinct) return left.distinct > right.distinct;
  return left.index > right.index;
}

function inferSequenceField(catalog: CatalogNumericField[][]): number | undefined {
  if (catalog.length < 3) return undefined;
  const fieldCount = Math.min(...catalog.map((fields) => fields.length));
  let best: CatalogFieldScore | undefined;
  for (let index = 0; index < fieldCount; ++index) {
    const timestamp = catalog[0][index].timestamp;
    if (catalog.some((fields) => fields[index].timestamp !== timestamp)) continue;
    const values = catalog.map((fields) => fields[index].value)
      .sort(compareNumber);
    const distinctValues = values.filter(
      (value, valueIndex) => valueIndex === 0 ||
        compareNumber(value, values[valueIndex - 1]) !== 0,
    );
    if (distinctValues.length < 2) continue;

    let density = distinctValues.length / catalog.length;
    if (!timestamp) {
      try {
        const integers = values.map((value) => BigInt(value));
        const minimum = integers.reduce((left, right) => left < right ? left : right);
        const maximum = integers.reduce((left, right) => left > right ? left : right);
        density = distinctValues.length / Number(maximum - minimum + 1n);
      } catch {
        density = 0;
      }
    }
    const score: CatalogFieldScore = {
      index,
      distinct: distinctValues.length,
      allUnique: distinctValues.length === catalog.length,
      integer: !timestamp,
      density,
    };
    if (!best || betterFieldScore(score, best)) best = score;
  }
  return best?.index;
}

export function createSequenceNameComparator(
  catalogNames: readonly string[],
): (left: string, right: string) => number {
  const catalog = catalogNames.map(catalogNumericFields);
  const fields = new Map(catalogNames.map((name) => [
    name,
    catalogNumericFields(name),
  ]));
  const primary = inferSequenceField(catalog);
  return (left, right) => {
    const leftFields = fields.get(left) ?? catalogNumericFields(left);
    const rightFields = fields.get(right) ?? catalogNumericFields(right);
    if (primary !== undefined && leftFields[primary] && rightFields[primary]) {
      const order = compareNumber(
        leftFields[primary].value,
        rightFields[primary].value,
      );
      if (order !== 0) return order;
    }
    return compareSequenceNames(left, right);
  };
}
