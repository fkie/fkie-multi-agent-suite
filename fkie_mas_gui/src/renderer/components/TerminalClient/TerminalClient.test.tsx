import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import TerminalClient from './TerminalClient';

describe('TerminalClient', () => {
  it('should render', () => {
    expect(
      render(
        <TerminalClient
          initialCommands={[]}
          tokenUrl=""
          wsUrl=""
          name=""
          invisibleTerminal={false}
          onIncomingData={() => {}}
        />
      )
    ).toBeTruthy();
  });
});
