import '@testing-library/jest-dom';
import { render } from '@testing-library/react';
import HostTreeViewItem from './HostTreeViewItem';

describe('HostTreeViewItem', () => {
  it('should render', () => {
    expect(
      render(
        <HostTreeViewItem
          bgColor="#fff"
          color="#000"
          labelText="Test"
          paddingLeft={100}
          key="text-key"
          nodeId="test-node-id"
          labelIcon={null}
          iconColor="#f0f"
          showMultipleScreen={false}
          showNoScreen={false}
          showGhostScreen={false}
          onStartClick={null}
          onStopClick={null}
          onRestartClick={null}
          startTooltipText="Start this node"
          stopTooltipText="Stop this node"
          restartTooltipText="Restart this node"
          tags={[]}
        />,
      ),
    ).toBeTruthy();
  });
});
