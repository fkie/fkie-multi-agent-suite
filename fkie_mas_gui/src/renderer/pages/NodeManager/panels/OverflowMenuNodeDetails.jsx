import MoreVertSharpIcon from '@mui/icons-material/MoreVert';
import OverflowMenu from '../../../components/UI/OverflowMenu';
import useLocalStorage from '../../../hooks/useLocalStorage';

function OverflowMenuNodeDetails({}) {
  const [showNodeInfo, setShowNodeInfo] = useLocalStorage(
    'NodesDetailsPanel:showNodeInfo',
    false,
  );
  const [showPublishers, setShowPublishers] = useLocalStorage(
    'NodesDetailsPanel:showPublishers',
    true,
  );
  const [showSubscribers, setShowSubscribers] = useLocalStorage(
    'NodesDetailsPanel:showSubscribers',
    true,
  );
  const [showServices, setShowServices] = useLocalStorage(
    'NodesDetailsPanel:showServices',
    false,
  );
  const [showConnections, setShowConnections] = useLocalStorage(
    'NodesDetailsPanel:showConnections',
    true,
  );

  const nodeDetailsOptions = [
    {
      name: 'Toggle Node Info',
      key: 'toggle-node-info',
      onClick: () => {
        setShowNodeInfo((prev) => !prev);
      },
    },
    {
      name: 'Toggle Publishers',
      key: 'toggle-publishers',
      onClick: () => {
        setShowPublishers((prev) => !prev);
      },
    },
    {
      name: 'Toggle Subscribers',
      key: 'toggle-subscribers',
      onClick: () => {
        setShowSubscribers((prev) => !prev);
      },
    },
    {
      name: 'Toggle Services',
      key: 'toggle-services',
      onClick: () => {
        setShowServices((prev) => !prev);
      },
    },
    {
      name: 'Toggle Connections',
      key: 'toggle-connections',
      onClick: () => {
        setShowConnections((prev) => !prev);
      },
    },
  ];

  return (
    <OverflowMenu
      icon={<MoreVertSharpIcon sx={{ fontSize: 16 }} />}
      options={nodeDetailsOptions}
      id="node-details-options"
    />
  );
}

OverflowMenuNodeDetails.defaultProps = {};

OverflowMenuNodeDetails.propTypes = {};

export default OverflowMenuNodeDetails;
