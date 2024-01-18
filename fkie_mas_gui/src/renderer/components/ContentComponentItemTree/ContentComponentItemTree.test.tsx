import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import ContentComponentItemTree from './ContentComponentItemTree';

describe('ContentComponentItemTree', () => {
  it('should render', () => {
    expect(render(<ContentComponentItemTree />)).toBeTruthy();
  });
});
