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

export function compareSequenceNames(left: string, right: string): number {
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
  signature: string;
}

interface CatalogFieldScore {
  signature: string;
  coverage: number;
  distinct: number;
  allUnique: boolean;
  integer: boolean;
  density: number;
}

function catalogNumericFields(name: string): CatalogNumericField[] {
  const tokens: SequenceToken[] = [];
  const scanner = new SequenceNameScanner(name);
  while (!scanner.empty) tokens.push(scanner.next());
  const fields: CatalogNumericField[] = [];
  const occurrences = new Map<string, number>();
  tokens.forEach((token, index) => {
    if (token.kind !== "number") return;
    const context = `${tokens[index - 1]?.value ?? "^"}\0${
      tokens[index + 1]?.value ?? "$"
    }`;
    const occurrence = occurrences.get(context) ?? 0;
    occurrences.set(context, occurrence + 1);
    fields.push({
      value: token.value,
      timestamp: token.value.includes("."),
      signature: `${context}\0${occurrence}`,
    });
  });
  return fields;
}

function betterFieldScore(
  left: CatalogFieldScore,
  right: CatalogFieldScore,
): boolean {
  if (left.integer !== right.integer) return left.integer;
  if (left.coverage !== right.coverage) return left.coverage > right.coverage;
  if (left.allUnique !== right.allUnique) return left.allUnique;
  if (left.density !== right.density) return left.density > right.density;
  if (left.distinct !== right.distinct) return left.distinct > right.distinct;
  return left.signature > right.signature;
}

function inferSequenceField(catalog: CatalogNumericField[][]): string | undefined {
  if (catalog.length < 3) return undefined;
  const bySignature = new Map<string, CatalogNumericField[]>();
  for (const fields of catalog) {
    for (const field of fields) {
      const matching = bySignature.get(field.signature) ?? [];
      matching.push(field);
      bySignature.set(field.signature, matching);
    }
  }
  let best: CatalogFieldScore | undefined;
  for (const [signature, matching] of bySignature) {
    if (matching.length < 3) continue;
    const timestamp = matching[0].timestamp;
    if (matching.some((field) => field.timestamp !== timestamp)) continue;
    const values = matching.map((field) => field.value).sort(compareNumber);
    const distinctValues = values.filter(
      (value, valueIndex) => valueIndex === 0 ||
        compareNumber(value, values[valueIndex - 1]) !== 0,
    );
    if (distinctValues.length < 2) continue;

    let density = distinctValues.length / matching.length;
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
      signature,
      coverage: matching.length,
      distinct: distinctValues.length,
      allUnique: distinctValues.length === matching.length,
      integer: !timestamp,
      density,
    };
    if (!best || betterFieldScore(score, best)) best = score;
  }
  return best?.signature;
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
    const leftPrimary = leftFields.find((field) => field.signature === primary);
    const rightPrimary = rightFields.find((field) => field.signature === primary);
    if (primary !== undefined && Boolean(leftPrimary) !== Boolean(rightPrimary)) {
      return leftPrimary ? -1 : 1;
    }
    if (leftPrimary && rightPrimary) {
      const order = compareNumber(
        leftPrimary.value,
        rightPrimary.value,
      );
      if (order !== 0) return order;
    }
    return compareSequenceNames(left, right);
  };
}
