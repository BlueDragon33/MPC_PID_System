import assert from 'node:assert/strict';

import { executeExperimentMatrixInWorker } from '../src/application/services/experimentService.js';

class SuccessfulWorker {
  static instances = [];

  constructor(url, options) {
    this.url = url;
    this.options = options;
    this.terminated = false;
    SuccessfulWorker.instances.push(this);
  }

  postMessage(message) {
    this.message = message;
    queueMicrotask(() => this.onmessage({
      data: {
        type: 'success',
        result: {
          requestedCases: 2,
          cases: [{ id: 'a' }, { id: 'b' }],
          groups: [{ presetId: 'baseline', cases: 2 }],
          overall: {},
        },
      },
    }));
  }

  terminate() {
    this.terminated = true;
  }
}

const options = { presetIds: ['baseline'], seeds: [1, 2] };
const result = await executeExperimentMatrixInWorker(options, SuccessfulWorker);
const worker = SuccessfulWorker.instances[0];
assert.equal(worker.options.type, 'module');
assert.equal(worker.message.type, 'run');
assert.deepEqual(worker.message.options, options);
assert.equal(result.requestedCases, 2);
assert.equal(worker.terminated, true, 'worker must terminate after a successful batch');

class FailedWorker extends SuccessfulWorker {
  postMessage() {
    queueMicrotask(() => this.onmessage({
      data: { type: 'error', error: { message: 'matrix failed safely' } },
    }));
  }
}

await assert.rejects(
  executeExperimentMatrixInWorker(options, FailedWorker),
  /matrix failed safely/,
);
assert.equal(SuccessfulWorker.instances.at(-1).terminated, true, 'worker must terminate after a failed batch');

class PostMessageFailedWorker extends SuccessfulWorker {
  postMessage() {
    throw new Error('message could not be sent');
  }
}

await assert.rejects(
  executeExperimentMatrixInWorker(options, PostMessageFailedWorker),
  /message could not be sent/,
);
assert.equal(SuccessfulWorker.instances.at(-1).terminated, true, 'worker must terminate when dispatch fails');

await assert.rejects(
  executeExperimentMatrixInWorker(options, null),
  /background experiment execution is unavailable/i,
);

console.log('experiment matrix worker contract: PASS');
