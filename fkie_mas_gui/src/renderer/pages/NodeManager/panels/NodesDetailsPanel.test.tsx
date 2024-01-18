import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import NodesDetailsPanel from './NodesDetailsPanel';

describe('NodesDetailsPanel', () => {
  it('should render', () => {
    expect(render(<NodesDetailsPanel />)).toBeTruthy();
  });
});
