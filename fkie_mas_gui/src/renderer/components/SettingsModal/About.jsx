import { Link, Stack, Typography } from '@mui/material';

import packageJson from '../../../../package.json';

function About() {
  return (
    <Stack paddingTop={2} spacing={0.2} sx={{ minHeight: 400 }} overflow="auto">
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: 'bold' }}>
          Version:
        </Typography>
        <Typography variant="body">{packageJson.version}</Typography>
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: 'bold' }}>
          License:
        </Typography>
        <Typography variant="body">{packageJson.license}</Typography>
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: 'bold' }}>
          Contributors:
        </Typography>
        <Typography variant="body">
          <Stack>
            {packageJson.contributors.map((item) => (
              <Typography key={`contributor-${item}`} variant="body">
                {item}
              </Typography>
            ))}
          </Stack>
        </Typography>
      </Stack>
      <Stack spacing={1} direction="row">
        <Typography variant="body" sx={{ fontWeight: 'bold' }}>
          Required additional software:
        </Typography>
        <Typography variant="body">
          <Stack>
            <Link
              href="https://github.com/fkie/multimaster_fkie"
              target="_blank"
              rel="noopener"
            >
              https://github.com/fkie/multimaster_fkie
            </Link>
            <Link
              href="https://github.com/tsl0922/ttyd"
              target="_blank"
              rel="noopener"
            >
              https://github.com/tsl0922/ttyd
            </Link>
          </Stack>
        </Typography>
      </Stack>{' '}
    </Stack>
  );
}

export default About;
