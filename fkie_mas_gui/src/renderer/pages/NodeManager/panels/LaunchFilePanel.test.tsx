import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import LaunchFilePanel from './LaunchFilePanel';
import { LaunchContent } from '../../../models';

describe('LaunchFilePanel', () => {
  it('should render', () => {
    const lc = new LaunchContent('', [], '', '', [], [], []);
    expect(render(<LaunchFilePanel launchContent={lc} />)).toBeTruthy();
  });
});
