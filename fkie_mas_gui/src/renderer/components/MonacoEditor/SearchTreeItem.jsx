import { alpha, styled } from '@mui/material/styles';
import PropTypes from 'prop-types';
import React from 'react';

import { Box, Chip, Stack, Typography } from '@mui/material';

import { TreeItem, treeItemClasses } from '@mui/x-tree-view';

const SearchTreeItemRoot = styled(TreeItem)(({ theme }) => ({
  color: theme.palette.text.secondary,
  [`& .${treeItemClasses.content}`]: {
    color: theme.palette.text.secondary,
    borderTopRightRadius: theme.spacing(2),
    borderBottomRightRadius: theme.spacing(2),
    paddingRight: theme.spacing(1),
    fontWeight: theme.typography.fontWeightMedium,
    '&.Mui-expanded': {
      fontWeight: theme.typography.fontWeightRegular,
    },
    '&:hover': {
      backgroundColor: theme.palette.action.hover,
    },
    '&.Mui-focused, &.Mui-selected, &.Mui-selected.Mui-focused': {
      backgroundColor: `var(--tree-view-bg-color, ${theme.palette.action.selected})`,
      color: 'var(--tree-view-color)',
    },
    [`& .${treeItemClasses.label}`]: {
      fontWeight: 'inherit',
      color: 'inherit',
      padding: theme.spacing(0),
    },
    [`& .${treeItemClasses.iconContainer}`]: {
      marginLeft: 0,
      marginRight: 0,
      padding: theme.spacing(0),
      width: 10,
    },
  },
  [`& .${treeItemClasses.group}`]: {
    marginLeft: 12,
    paddingLeft: 5,
    // [`& .${treeItemClasses.content}`]: {
    //   paddingLeft: theme.spacing(0),
    // },
    borderLeft: `1px dashed ${alpha(theme.palette.text.primary, 0.4)}`,
    // borderColor: black[50],
  },
}));

const SearchFileTreeItem = React.forwardRef(function SearchFileTreeItem(
  { labelIcon, labelText, labelInfo, labelCount, ...other },
  ref,
) {
  return (
    <SearchTreeItemRoot
      label={
        <Stack direction="column">
          <Stack
            spacing={1}
            direction="row"
            alignItems={'center'}
            marginLeft={1}
          >
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}

            <Typography
              flexGrow={1}
              variant="body2"
              sx={{ fontWeight: 'inherit' }}
            >
              {labelText}
            </Typography>

            {labelInfo && (
              <Typography variant="caption" color="inherit">
                [{labelInfo}]
              </Typography>
            )}
            {labelCount > 0 && (
              <Chip
                size="small"
                color={'default'}
                label={labelCount}
                sx={{ fontSize: '0.6em', height: 'auto', width: 'auto' }}
              />
            )}
          </Stack>
        </Stack>
      }
      style={{ marginTop: 1 }}
      {...other}
      ref={ref}
    />
  );
});

SearchFileTreeItem.defaultProps = {
  labelIcon: null,
  labelText: '',
  labelInfo: '',
  labelCount: 0,
};

SearchFileTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  labelInfo: PropTypes.string,
  labelCount: PropTypes.number,
};

const SearchResultTreeItem = React.forwardRef(function SearchResultTreeItem(
  { labelIcon, labelText, labelInfo, onClick, ...other },
  ref,
) {
  return (
    <SearchTreeItemRoot
      label={
        <Stack direction="column">
          <Stack spacing={1} direction="row" alignItems={'center'}>
            {labelIcon && (
              <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
            )}

            <Typography variant="body2" sx={{ fontWeight: 'inherit' }}>
              {labelText}
            </Typography>

            {labelInfo && (
              <Typography noWrap variant="caption" color="inherit">
                {labelInfo}
              </Typography>
            )}
          </Stack>
        </Stack>
      }
      onClick={(event) => {
        if (onClick) {
          onClick(event);
        }
      }}
      {...other}
      ref={ref}
    />
  );
});

// const SearchTreeItem = React.forwardRef(function TopicTreeItem(
//   {
//     labelRoot,
//     labelIcon,
//     labelInfo,
//     labelCount,
//     labelText,
//     requestData,
//     searchInfo,
//     ...other
//   },
//   ref,
// ) {
//   const logCtx = useContext(LoggingContext);
//   const navCtx = useContext(NavigationContext);
//   const settingsCtx = useContext(SettingsContext);
//   const [label, setLabel] = useState(labelText);
//   const [showExtendedInfo, setShowExtendedInfo] = useState(false);

//   const getHostStyle = () => {
//     if (topicInfo?.providerName && settingsCtx.get('colorizeHosts')) {
//       return {
//         flexGrow: 1,
//         alignItems: 'center',
//         borderLeftStyle: 'outset',
//         borderLeftColor: colorFromHostname(topicInfo?.providerName),
//         borderLeftWidth: '0.6em',
//       };
//     }
//     return { flexGrow: 1, alignItems: 'center' };
//   };

//   useEffect(() => {
//     if (!labelRoot) return;
//     if (!topicInfo) return;

//     if (topicInfo?.name === labelRoot) {
//       setLabel(topicInfo.providerName);
//     } else {
//       setLabel(labelText.slice(labelRoot.length + 1));
//     }
//   }, [labelRoot, topicInfo]);

//   return (
//     <SearchTreeItemRoot
//       label={
//         <Stack direction="column">
//           <Box
//             sx={{
//               display: 'flex',
//               alignItems: 'center',
//               p: 0.3,
//               pr: 0,
//             }}
//           >
//             {labelIcon && (
//               <Box component={labelIcon} color="inherit" sx={{ mr: 1 }} />
//             )}
//             <Stack spacing={1} direction="row" sx={getHostStyle()}>
//               <Typography
//                 variant="body2"
//                 sx={{ fontWeight: 'inherit' }}
//                 onClick={(e) => {
//                   if (e.detail === 2) {
//                     navigator.clipboard.writeText(labelText);
//                     logCtx.success(`${labelText} copied!`);
//                   }
//                 }}
//               >
//                 {label}
//               </Typography>
//               {/* {requestData && <CircularProgress size="1em" />} */}
//             </Stack>
//             <Stack
//               direction="row"
//               spacing={1}
//               sx={{
//                 alignItems: 'center',
//               }}
//             >
//               {labelInfo && (
//                 <Typography variant="caption" color="inherit" padding={0.5}>
//                   [{labelInfo}]
//                 </Typography>
//               )}
//               {labelCount > 0 && (
//                 // <Tag text={labelCount} color="default" copyButton={false}></Tag>
//                 <Typography variant="caption" color="inherit" padding={0.5}>
//                   [{labelCount}]
//                 </Typography>
//               )}
//               {topicInfo && (
//                 <Stack direction="row" spacing={1}>
//                   <Chip
//                     size="small"
//                     title="publishers"
//                     // showZero={true}
//                     color={
//                       topicInfo.publishers.length > 0 ? 'default' : 'warning'
//                     }
//                     label={topicInfo.publishers.length}
//                     onClick={(event) => {
//                       setShowExtendedInfo(!showExtendedInfo);
//                       event.stopPropagation();
//                     }}
//                   />

//                   <Chip
//                     size="small"
//                     title="subscribers"
//                     // showZero={true}
//                     color={
//                       topicInfo.subscribers.length > 0 ? 'default' : 'warning'
//                     }
//                     label={topicInfo.subscribers.length}
//                     onClick={(event) => {
//                       setShowExtendedInfo(!showExtendedInfo);
//                       event.stopPropagation();
//                     }}
//                   />
//                 </Stack>
//               )}
//             </Stack>
//           </Box>
//           {showExtendedInfo && topicInfo && (
//             <Stack paddingLeft={3}>
//               <Typography fontWeight="bold" fontSize="small">
//                 Publisher [{topicInfo.publishers.length}]:
//               </Typography>
//               {topicInfo.publishers.map((item) => {
//                 const pubNodeName = item.split('-', 2).slice(-1).join('-');
//                 return (
//                   <Stack key={item} paddingLeft={3} direction="row">
//                     <Typography
//                       fontSize="small"
//                       onClick={() => {
//                         navCtx.setSelectedNodes([
//                           `${topicInfo.providerId}${item.replaceAll('/', '.')}`,
//                         ]);
//                       }}
//                     >
//                       {pubNodeName}
//                     </Typography>
//                     <CopyButton value={pubNodeName} />
//                   </Stack>
//                 );
//               })}
//               <Typography fontWeight="bold" fontSize="small">
//                 Subscriber [{topicInfo.subscribers.length}]:
//               </Typography>
//               {topicInfo.subscribers.map((item) => {
//                 const subNodeName = item.split('-', 2).slice(-1).join('-');
//                 return (
//                   <Stack key={item} paddingLeft={3} direction="row">
//                     <Typography
//                       fontSize="small"
//                       onClick={() => {
//                         navCtx.setSelectedNodes([
//                           `${topicInfo.providerId}${item.replaceAll('/', '.')}`,
//                         ]);
//                       }}
//                     >
//                       {subNodeName}
//                     </Typography>
//                     <CopyButton value={subNodeName} />
//                   </Stack>
//                 );
//               })}
//             </Stack>
//           )}
//         </Stack>
//       }
//       {...other}
//       ref={ref}
//     />
//   );
// });

SearchResultTreeItem.defaultProps = {
  labelIcon: null,
  labelText: '',
  labelInfo: '',
  onClick: null,
};

SearchResultTreeItem.propTypes = {
  labelIcon: PropTypes.object,
  labelText: PropTypes.string,
  labelInfo: PropTypes.string,
  onClick: PropTypes.func,
};

export { SearchFileTreeItem, SearchResultTreeItem };
